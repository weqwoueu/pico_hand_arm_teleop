"""XRoboToolkit SDK 底层封装层。

本模块对 ``xrobotoolkit_sdk`` 进行轻量封装，目的：

1. 统一接口：对外只暴露 Pose / Trigger / Grip / Button 这些业务关心的量；
2. 安全取值：SDK 未初始化、未收到数据、字段缺失等异常场景全部退化成零值/False，
   保证上层 100Hz 主循环不会因偶发异常而中断；
3. 可替换：SDK 不在环境里时（开发机无 PICO），上层仍可正常跑通逻辑（退化为默认位姿）。
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass, field
from typing import Callable, Optional

import numpy as np

logger = logging.getLogger(__name__)

try:  # SDK 是可选依赖，开发机可以没有
    import xrobotoolkit_sdk as xrt  # type: ignore
except Exception as _exc:  # noqa: BLE001
    xrt = None  # type: ignore[assignment]
    logger.warning("未能导入 xrobotoolkit_sdk，将使用空值占位: %s", _exc)


# 7 维 Pose: [x, y, z, qx, qy, qz, qw]
PoseArray = np.ndarray


def _identity_pose() -> PoseArray:
    return np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0], dtype=np.float64)


@dataclass
class ControllerSnapshot:
    """一次采样快照，方便上层一次性拿走所有需要的量。

    PICO 4 控制器按键拓扑：
    - 右手柄：A / B / 右扳机 / 右握把 / 右菜单
    - 左手柄：X / Y / 左扳机 / 左握把 / 左菜单
    """

    headset_pose: PoseArray
    right_pose: PoseArray
    right_trigger: float
    right_grip: float
    button_a: bool
    button_menu: bool
    timestamp: float
    # 左手柄（向后兼容：旧调用方继续读 right_*，新代码可读 left_*）
    left_pose: PoseArray = field(default_factory=lambda: _identity_pose())
    left_trigger: float = 0.0
    left_grip: float = 0.0
    button_x: bool = False
    left_button_menu: bool = False


class XrClient:
    """XRoboToolkit SDK 的安全封装。"""

    def __init__(self) -> None:
        self._initialized = False
        self._lock = threading.Lock()

    def init(self) -> None:
        """初始化 SDK；重复调用幂等。"""
        if xrt is None:
            logger.warning("xrobotoolkit_sdk 未安装，XrClient 以 dummy 模式运行。")
            return
        with self._lock:
            if self._initialized:
                return
            # TODO: 如果真实 SDK 需要额外参数（如配置文件路径），请在此处补全
            xrt.init()
            self._initialized = True
            logger.info("XRoboToolkit SDK 初始化完成。")

    def close(self) -> None:
        """释放 SDK 资源。"""
        if xrt is None:
            return
        with self._lock:
            if not self._initialized:
                return
            try:
                close_fn = getattr(xrt, "close", None) or getattr(xrt, "shutdown", None)
                if callable(close_fn):
                    close_fn()
            except Exception as exc:  # noqa: BLE001
                logger.warning("xrt 关闭时异常: %s", exc)
            self._initialized = False
            logger.info("XRoboToolkit SDK 已关闭。")

    # ---------- 通用安全读取 ----------

    def _safe_call(self, getter: Optional[Callable[[], object]], default):
        if xrt is None or not self._initialized or getter is None:
            return default
        try:
            value = getter()
            if value is None:
                return default
            return value
        except Exception as exc:  # noqa: BLE001
            logger.debug("SDK 调用异常 %s: %s", getattr(getter, "__name__", "?"), exc)
            return default

    def _safe_pose(self, getter: Optional[Callable[[], object]]) -> PoseArray:
        raw = self._safe_call(getter, None)
        if raw is None:
            return _identity_pose()
        try:
            arr = np.asarray(raw, dtype=np.float64).reshape(-1)
            if arr.size != 7:
                return _identity_pose()
            # 兜底：PC Service 异步启动期间、或偶发掉帧，SDK 会返回全零 pose
            # (qx=qy=qz=qw=0)，下游 scipy.from_quat 会抛 "zero norm quaternions"。
            # 这里检测零范四元数并退化为 identity，保证 100Hz 主循环不会被一帧空数据打断。
            q_norm = float(np.linalg.norm(arr[3:7]))
            if not np.isfinite(q_norm) or q_norm < 1e-6:
                return _identity_pose()
            return arr
        except Exception:  # noqa: BLE001
            return _identity_pose()

    # ---------- 具体字段 ----------

    def get_headset_pose(self) -> PoseArray:
        return self._safe_pose(getattr(xrt, "get_headset_pose", None) if xrt else None)

    def get_right_controller_pose(self) -> PoseArray:
        return self._safe_pose(
            getattr(xrt, "get_right_controller_pose", None) if xrt else None
        )

    def get_right_trigger(self) -> float:
        value = self._safe_call(
            getattr(xrt, "get_right_trigger", None) if xrt else None, 0.0
        )
        try:
            return float(np.clip(value, 0.0, 1.0))
        except Exception:  # noqa: BLE001
            return 0.0

    def get_right_grip(self) -> float:
        value = self._safe_call(
            getattr(xrt, "get_right_grip", None) if xrt else None, 0.0
        )
        try:
            return float(np.clip(value, 0.0, 1.0))
        except Exception:  # noqa: BLE001
            return 0.0

    def get_right_button_a(self) -> bool:
        # 不同 SDK 版本的命名差异兼容尝试
        for name in ("get_A_button", "get_right_button_a", "get_right_a_button"):
            if xrt is not None and hasattr(xrt, name):
                return bool(self._safe_call(getattr(xrt, name), False))
        return False

    def get_right_button_menu(self) -> bool:
        for name in ("get_right_menu_button", "get_menu_button"):
            if xrt is not None and hasattr(xrt, name):
                return bool(self._safe_call(getattr(xrt, name), False))
        return False

    # ---------- 左手柄 ----------

    def get_left_controller_pose(self) -> PoseArray:
        return self._safe_pose(
            getattr(xrt, "get_left_controller_pose", None) if xrt else None
        )

    def get_left_trigger(self) -> float:
        value = self._safe_call(
            getattr(xrt, "get_left_trigger", None) if xrt else None, 0.0
        )
        try:
            return float(np.clip(value, 0.0, 1.0))
        except Exception:  # noqa: BLE001
            return 0.0

    def get_left_grip(self) -> float:
        value = self._safe_call(
            getattr(xrt, "get_left_grip", None) if xrt else None, 0.0
        )
        try:
            return float(np.clip(value, 0.0, 1.0))
        except Exception:  # noqa: BLE001
            return 0.0

    def get_left_button_x(self) -> bool:
        for name in ("get_X_button", "get_left_button_x", "get_left_x_button"):
            if xrt is not None and hasattr(xrt, name):
                return bool(self._safe_call(getattr(xrt, name), False))
        return False

    def get_left_button_menu(self) -> bool:
        for name in ("get_left_menu_button",):
            if xrt is not None and hasattr(xrt, name):
                return bool(self._safe_call(getattr(xrt, name), False))
        return False

    # ---------- 聚合采样 ----------

    def snapshot(self) -> ControllerSnapshot:
        return ControllerSnapshot(
            headset_pose=self.get_headset_pose(),
            right_pose=self.get_right_controller_pose(),
            right_trigger=self.get_right_trigger(),
            right_grip=self.get_right_grip(),
            button_a=self.get_right_button_a(),
            button_menu=self.get_right_button_menu(),
            timestamp=time.time(),
            left_pose=self.get_left_controller_pose(),
            left_trigger=self.get_left_trigger(),
            left_grip=self.get_left_grip(),
            button_x=self.get_left_button_x(),
            left_button_menu=self.get_left_button_menu(),
        )
