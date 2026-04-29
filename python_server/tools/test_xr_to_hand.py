"""真实 PICO 手柄 trigger/grip -> 真实 Revo2 灵巧手 端到端最小闭环脚本。

这是 ``test_hand_mapping.py`` 的 "把模拟余弦波换成真 PICO 输入" 版本。
其余管线（``compute_revo2_targets`` + ``to_sdk_positions`` + libstark 100Hz）保持一致。

验收动作：

- 按 **右手柄 trigger** -> 食/中/无名/小 四指同步屈曲；
- 按 **右手柄 grip**    -> 拇指屈伸（``Thumb Flex``，槽位 0）；
- 两个都按              -> 五指齐握；
- 都松开                -> 手全伸开。
- 拇指对掌 / 内外收（``Thumb Aux``，槽位 1）保持 ``THUMB_OPPOSITION_DEFAULT`` 的常量（不随手柄变化）。

前置：

1. PC Service 已在跑（``/opt/apps/roboticsservice/runService.sh``）；
2. PICO 头显客户端 APK 已连上 PC；
3. Revo2 已上电、USB-RS485 就位、``/dev/ttyUSB*`` 有读写权限；
4. 环境变量已通过 ``python_server/run.sh`` 注入 ``LD_LIBRARY_PATH``。

用法::

    cd python_server
    ./run.sh -m tools.test_xr_to_hand
"""

from __future__ import annotations

import asyncio
import logging
import sys
import time
from pathlib import Path

# 兼容两种启动方式：
#   1) ./run.sh -m tools.test_xr_to_hand     （推荐）
#   2) python tools/test_xr_to_hand.py       （直跑，此时下面这段 sys.path 兜底）
_PKG_ROOT = Path(__file__).resolve().parent.parent
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

try:
    from bc_stark_sdk import main_mod as libstark  # type: ignore
except Exception as exc:  # noqa: BLE001
    print(f"[ERROR] 未安装 bc-stark-sdk: {exc}", file=sys.stderr)
    print("    请在 python_server 下执行: uv add bc-stark-sdk colorlog", file=sys.stderr)
    sys.exit(1)

from core.mapping_utils import (
    THUMB_OPPOSITION_DEFAULT,
    compute_revo2_targets,
    to_sdk_positions,
)
from core.xr_client import XrClient


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger("test_xr_to_hand")


CTRL_HZ = 100
CTRL_DT = 1.0 / CTRL_HZ

# 打印节流：100Hz 太刷屏，降到 ~5Hz 供肉眼观察
LOG_EVERY_N_TICKS = 20


async def _auto_open_revo2():
    """复刻 test_hand_mapping 的 Revo2 自动探测 + Modbus 打开。"""
    logger.info("自动探测 Revo2 设备...")
    protocol, port_name, baudrate, slave_id = await libstark.auto_detect_modbus_revo2(
        None, True
    )
    assert protocol == libstark.StarkProtocolType.Modbus, "本脚本仅支持 Modbus"
    logger.info(
        "探测成功: port=%s baudrate=%s slave_id=0x%x", port_name, baudrate, slave_id
    )
    client = await libstark.modbus_open(port_name, baudrate)
    return client, slave_id


async def main() -> None:
    # 1) 先把 PICO 那头拉起来。SDK 未装时 XrClient 会降级成 dummy（全零），
    #    那种情况下不会崩，但手也不会动 —— 正好一眼就看出是上游没通。
    xr = XrClient()
    xr.init()
    logger.info("XrClient 已初始化。")

    # 2) 再开 Revo2（放后面是因为 PICO 初始化快、Revo2 探测可能等几秒）
    revo2, slave_id = await _auto_open_revo2()
    try:
        info = await revo2.get_device_info(slave_id)
        logger.info("Revo2 设备信息: %s", getattr(info, "description", info))

        # 归一化模式：位置量程 0~1000 (0.1% 精度)
        await revo2.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized)
        speeds = [1000] * 6

        logger.info(
            "开始 PICO -> Revo2 实机遥操 @ %dHz。按 Ctrl+C 退出。", CTRL_HZ
        )
        logger.info(
            "验收：trigger=食+中+无名+小 四指屈曲 / grip=拇指屈伸(ThumbFlex) / 拇指内外收(ThumbAux) 固定 %.0f%%",
            THUMB_OPPOSITION_DEFAULT * 100,
        )

        tick = 0
        while True:
            # 真实 PICO 输入（snapshot 同步返回，几十 us 级，不阻塞 asyncio loop）
            snap = xr.snapshot()
            trigger = float(snap.right_trigger)
            grip = float(snap.right_grip)

            targets = compute_revo2_targets(
                trigger=trigger,
                grip=grip,
                thumb_opposition=THUMB_OPPOSITION_DEFAULT,
            )
            positions = to_sdk_positions(targets)

            await revo2.set_finger_positions_and_speeds(slave_id, positions, speeds)

            if tick % LOG_EVERY_N_TICKS == 0:
                logger.info(
                    "trig=%.2f grip=%.2f A=%s menu=%s -> sdk=%s",
                    trigger,
                    grip,
                    snap.button_a,
                    snap.button_menu,
                    positions,
                )

            tick += 1
            await asyncio.sleep(CTRL_DT)
    finally:
        # 清理顺序：先断 Revo2（释放串口），再关 XrClient
        try:
            libstark.modbus_close(revo2)
        except Exception as exc:  # noqa: BLE001
            logger.warning("Revo2 断开异常: %s", exc)
        try:
            xr.close()
        except Exception as exc:  # noqa: BLE001
            logger.warning("XrClient 关闭异常: %s", exc)
        logger.info("退出清理完成。")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("用户中断。")
        sys.exit(0)
