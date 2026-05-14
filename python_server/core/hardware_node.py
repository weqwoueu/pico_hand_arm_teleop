"""硬件执行层：天机 Marvin M6CCS 机械臂 + BrainCo Revo2 灵巧手。

暴露给上层（``main.py``、``PicoStreamer``）的统一接口：

- ``servo_cartesian(T_matrix)``：以笛卡尔位姿形式控制机械臂末端；
- ``set_finger_positions(fingers)``：控制 Revo2 六个自由度（归一化）。

本文件混合了同步主循环（100Hz）和 libstark 的异步接口：

- ``TianjiArmDriver`` 当前仍是 Dummy（真机 SDK 未集成）；
- ``Revo2HandDriver`` 为真实驱动：内部启动一个后台 asyncio 事件循环线程，
  主循环线程通过 ``call_soon_threadsafe`` 投递"最新目标"，后台线程以
  *latest-wins* 的方式把最新目标下发给硬件。命令不会在队列里累积，
  如果硬件临时卡顿，丢掉的是中间帧，保留的是最新意图。
"""

from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass
from typing import List, Optional

import numpy as np

from .mapping_utils import Revo2FingerTargets, to_sdk_positions

logger = logging.getLogger(__name__)

DEFAULT_REVO2_BAUDRATE = 460800
DEFAULT_REVO2_SLAVE_ID = 0x7E


# ---------- 机械臂 ----------


@dataclass
class TianjiConfig:
    """天机机械臂连接配置。"""

    ip: str = "192.168.1.10"
    port: int = 8080
    # TODO: 填入真实连接参数（如控制模式、速度上限等）


class TianjiArmDriver:
    """Tianji Marvin M6CCS 机械臂 Dummy 驱动。

    真实实现需要：
    1. 通过 TCP/串口连接控制器；
    2. 切换到笛卡尔伺服模式；
    3. ``servo_cartesian`` 在 100Hz 节拍下发目标位姿。
    """

    def __init__(self, config: Optional[TianjiConfig] = None) -> None:
        self._config = config or TianjiConfig()
        self._connected = False
        self._current_T = np.eye(4, dtype=np.float64)
        self._current_T[:3, 3] = [0.40, 0.0, 0.45]

    def connect(self) -> None:
        # TODO: 替换为真实 SDK 的连接与上电逻辑
        logger.info(
            "[Dummy] 连接天机机械臂 ip=%s port=%d", self._config.ip, self._config.port
        )
        self._connected = True

    def disconnect(self) -> None:
        # TODO: 替换为真实 SDK 的断电与释放逻辑
        logger.info("[Dummy] 断开天机机械臂")
        self._connected = False

    def get_current_ee_pose(self) -> np.ndarray:
        # TODO: 替换为真实 SDK 的末端位姿读取
        return self._current_T.copy()

    def servo_cartesian(self, T_target: np.ndarray) -> None:
        if not self._connected:
            logger.warning("机械臂未连接，忽略 servo_cartesian 调用。")
            return
        if T_target.shape != (4, 4):
            raise ValueError(f"T_target 形状应为 (4,4)，实际为 {T_target.shape}")
        # TODO: 替换为真实 SDK 的笛卡尔伺服接口
        self._current_T = T_target.copy()


# ---------- 灵巧手 ----------


@dataclass
class Revo2Config:
    """Revo2 灵巧手连接配置。

    当 ``port`` 为 ``None`` 时走 ``auto_detect_modbus_revo2`` 自动探测，
    适合开发阶段"插上就用"。给出具体值则强制用指定端口/波特率/从站号。
    """

    port: Optional[str] = None
    baudrate: Optional[int] = None
    slave_id: Optional[int] = None
    # 下发速度（0~1000），默认满速
    speed: int = 1000
    # 退出时先把手指归零再关连接，避免抓着物体卡住
    safe_release_on_disconnect: bool = True
    # connect 等待后台线程就绪的超时（秒）
    connect_timeout: float = 10.0


class Revo2HandDriver:
    """BrainCo Revo2 灵巧手真实驱动（同步 facade + 后台 asyncio 线程）。

    设计：

    - ``connect()`` 启动后台线程，后台线程内建 asyncio loop，完成
      ``auto_detect`` / ``modbus_open`` / ``set_finger_unit_mode``，
      主线程阻塞直到收到 ``_ready`` 事件（或 timeout / 连接异常）。
    - ``set_finger_positions(fingers)`` 同步、非阻塞：把 6 维目标位置写入共享变量，
      然后 ``call_soon_threadsafe`` 唤醒后台循环。后台始终发送"最新值"，命令不累积。
    - ``disconnect()`` 负责安全归零（可选）+ 关闭 libstark 连接 + join 后台线程。
    """

    def __init__(self, config: Optional[Revo2Config] = None) -> None:
        self._config = config or Revo2Config()

        # 线程 / loop 句柄
        self._thread: Optional[threading.Thread] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        # 同步原语
        self._ready = threading.Event()
        self._stop = False
        self._connect_error: Optional[BaseException] = None

        # latest-wins 命令槽位（主线程写、后台线程读）
        self._cmd_lock = threading.Lock()
        self._latest_positions: Optional[List[int]] = None

        # 在后台 loop 里创建，跨线程用 call_soon_threadsafe 唤醒
        self._new_cmd_event: Optional[asyncio.Event] = None

        # libstark 客户端句柄（后台线程持有）
        self._client = None
        self._slave_id: Optional[int] = None

        # 速度向量（每帧复用）
        self._speeds = [self._config.speed] * 6

    # ---------- 对外同步 API ----------

    def connect(self) -> None:
        if self._thread is not None:
            raise RuntimeError("Revo2HandDriver 已连接，不要重复 connect()")

        self._thread = threading.Thread(
            target=self._thread_main, name="Revo2AsyncLoop", daemon=True
        )
        self._thread.start()

        if not self._ready.wait(timeout=self._config.connect_timeout):
            raise TimeoutError(
                f"Revo2 连接超时 (> {self._config.connect_timeout:.1f}s)"
            )
        if self._connect_error is not None:
            exc = self._connect_error
            self._thread.join(timeout=1.0)
            self._thread = None
            raise exc

        logger.info("Revo2HandDriver 已就绪（slave_id=0x%x）。", self._slave_id or 0)

    def set_finger_positions(self, fingers: Revo2FingerTargets) -> None:
        """主循环入口：把目标归一化量写到共享槽位，非阻塞返回。"""
        positions = to_sdk_positions(fingers)
        with self._cmd_lock:
            self._latest_positions = positions

        loop = self._loop
        ev = self._new_cmd_event
        if loop is not None and ev is not None and not loop.is_closed():
            loop.call_soon_threadsafe(ev.set)

    def disconnect(self) -> None:
        if self._thread is None:
            return
        logger.info("断开 Revo2 ...")

        if self._config.safe_release_on_disconnect and self._ready.is_set():
            try:
                with self._cmd_lock:
                    self._latest_positions = [0, 0, 0, 0, 0, 0]
                loop = self._loop
                ev = self._new_cmd_event
                if loop is not None and ev is not None and not loop.is_closed():
                    loop.call_soon_threadsafe(ev.set)
                time.sleep(0.2)
            except Exception as exc:  # noqa: BLE001
                logger.warning("归零 Revo2 时异常: %s", exc)

        self._stop = True
        loop = self._loop
        ev = self._new_cmd_event
        if loop is not None and ev is not None and not loop.is_closed():
            loop.call_soon_threadsafe(ev.set)

        self._thread.join(timeout=3.0)
        if self._thread.is_alive():
            logger.warning("Revo2 后台线程未在 3s 内退出，将被进程退出时回收。")
        self._thread = None
        self._loop = None
        logger.info("Revo2 已断开。")

    # ---------- 后台线程 ----------

    def _thread_main(self) -> None:
        loop = asyncio.new_event_loop()
        self._loop = loop
        asyncio.set_event_loop(loop)
        try:
            loop.run_until_complete(self._async_main())
        except BaseException as exc:  # noqa: BLE001
            if not self._ready.is_set():
                self._connect_error = exc
                self._ready.set()
            else:
                logger.exception("Revo2 后台线程异常退出: %s", exc)
        finally:
            try:
                loop.close()
            except Exception:  # noqa: BLE001
                pass

    async def _async_main(self) -> None:
        try:
            from bc_stark_sdk import main_mod as libstark  # type: ignore
        except Exception as exc:  # noqa: BLE001
            self._connect_error = RuntimeError(
                f"未安装 bc-stark-sdk: {exc}。请在 python_server 下执行: uv add bc-stark-sdk"
            )
            self._ready.set()
            return

        self._new_cmd_event = asyncio.Event()

        # ---- 1. 连接设备 ----
        try:
            if self._config.port is None:
                logger.info("自动探测 Revo2 设备（可能需要几秒）...")
                protocol, port_name, baudrate, slave_id = (
                    await libstark.auto_detect_modbus_revo2(None, True)
                )
                assert protocol == libstark.StarkProtocolType.Modbus, (
                    "目前仅支持 Modbus 协议"
                )
            else:
                port_name = self._config.port
                baudrate = self._coerce_baudrate(
                    libstark,
                    self._config.baudrate or DEFAULT_REVO2_BAUDRATE,
                )
                slave_id = self._config.slave_id or DEFAULT_REVO2_SLAVE_ID

            logger.info(
                "打开 Revo2: port=%s baudrate=%s slave_id=0x%x",
                port_name,
                baudrate,
                slave_id,
            )
            client = await libstark.modbus_open(port_name, baudrate)
            await client.set_finger_unit_mode(
                slave_id, libstark.FingerUnitMode.Normalized
            )
            self._client = client
            self._slave_id = slave_id
        except BaseException as exc:  # noqa: BLE001
            self._connect_error = exc
            self._ready.set()
            return

        self._ready.set()

        # ---- 2. 命令下发循环（latest-wins）----
        try:
            while not self._stop:
                await self._new_cmd_event.wait()
                self._new_cmd_event.clear()
                if self._stop:
                    break

                with self._cmd_lock:
                    positions = self._latest_positions
                    self._latest_positions = None

                if positions is None:
                    continue

                try:
                    await client.set_finger_positions_and_speeds(
                        slave_id, positions, self._speeds
                    )
                except Exception as exc:  # noqa: BLE001
                    logger.warning("Revo2 下发失败: %s", exc)
        finally:
            try:
                libstark.modbus_close(client)
            except Exception as exc:  # noqa: BLE001
                logger.warning("Revo2 关闭连接异常: %s", exc)

    @staticmethod
    def _coerce_baudrate(libstark, baudrate):  # noqa: ANN001
        if not isinstance(baudrate, int):
            return baudrate
        baudrate_cls = libstark.Baudrate
        if hasattr(baudrate_cls, "from_int"):
            try:
                return baudrate_cls.from_int(baudrate)
            except Exception:  # noqa: BLE001
                pass
        baudrate_map = {
            19200: baudrate_cls.Baud19200,
            57600: baudrate_cls.Baud57600,
            115200: baudrate_cls.Baud115200,
            460800: baudrate_cls.Baud460800,
            1000000: baudrate_cls.Baud1Mbps,
            2000000: baudrate_cls.Baud2Mbps,
            5000000: baudrate_cls.Baud5Mbps,
        }
        try:
            return baudrate_map[baudrate]
        except KeyError as exc:
            supported = ", ".join(str(k) for k in sorted(baudrate_map))
            raise ValueError(f"不支持的 Revo2 baudrate={baudrate}，支持: {supported}") from exc


# ---------- 组合节点 ----------


class TianjiRevoHardwareNode:
    """机械臂 + 灵巧手的统一硬件节点。"""

    def __init__(
        self,
        arm: Optional[TianjiArmDriver] = None,
        hand: Optional[Revo2HandDriver] = None,
    ) -> None:
        self.arm = arm or TianjiArmDriver()
        self.hand = hand or Revo2HandDriver()

    def connect(self) -> None:
        self.arm.connect()
        self.hand.connect()

    def disconnect(self) -> None:
        try:
            self.arm.disconnect()
        finally:
            self.hand.disconnect()

    def get_current_ee_pose(self) -> np.ndarray:
        return self.arm.get_current_ee_pose()

    def servo_cartesian(self, T_target: np.ndarray) -> None:
        self.arm.servo_cartesian(T_target)

    def set_finger_positions(self, fingers: Revo2FingerTargets) -> None:
        self.hand.set_finger_positions(fingers)
