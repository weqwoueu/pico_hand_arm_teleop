"""真实 PICO 手柄 trigger/grip -> 真实 Revo2 灵巧手 端到端最小闭环脚本。

这是 ``test_hand_mapping.py`` 的 "把模拟余弦波换成真 PICO 输入" 版本。
其余管线（``compute_revo2_targets`` + ``Revo2HandDriver`` + 100Hz）保持一致。

验收动作（默认右手柄，可用 ``--controller left`` 切到左手柄）：

- 按所选手柄 trigger -> 食/中/无名/小 四指同步屈曲；
- 按所选手柄 grip    -> 拇指屈伸（``Thumb Flex``，槽位 0）；
- 两个都按              -> 五指齐握；
- 都松开                -> 手全伸开。
- 拇指对掌 / 内外收（``Thumb Aux``，槽位 1）保持 ``THUMB_OPPOSITION_DEFAULT`` 的常量（不随手柄变化）。

前置：

1. PC Service 已在跑（``/opt/apps/roboticsservice/runService.sh``）；
2. PICO 头显客户端 APK 已连上 PC；
3. Revo2 已上电、USB-RS485 就位、``/dev/ttyUSB*`` 有读写权限；
4. 环境变量已通过 ``python_server/run.sh`` 注入 ``LD_LIBRARY_PATH``。

用法::
    sudo chmod 666 /dev/ttyUSB0
    cd python_server
    ./run.sh -m tools.test_xr_to_hand --hand-port /dev/ttyUSB0
    ./run.sh -m tools.test_xr_to_hand --controller left --hand-side left --hand-port /dev/ttyUSB0
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import sys
from pathlib import Path

# 兼容两种启动方式：
#   1) ./run.sh -m tools.test_xr_to_hand     （推荐）
#   2) python tools/test_xr_to_hand.py       （直跑，此时下面这段 sys.path 兜底）
_PKG_ROOT = Path(__file__).resolve().parent.parent
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from core.hand_core import Revo2HandConfig, Revo2HandDriver
from core.mapping_utils import (
    THUMB_OPPOSITION_DEFAULT,
    compute_revo2_targets,
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


def _pick_hand_inputs(snap, side: str) -> tuple[float, float]:  # noqa: ANN001
    if side == "left":
        return float(snap.left_trigger), float(snap.left_grip)
    if side == "right":
        return float(snap.right_trigger), float(snap.right_grip)
    raise ValueError("side 只能是 'left' 或 'right'")


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="PICO 手柄 -> Revo2 单手验收")
    ap.add_argument(
        "--controller",
        choices=("left", "right"),
        default="right",
        help="用于控制 Revo2 的 PICO 手柄；默认 right，保持之前验收逻辑",
    )
    ap.add_argument(
        "--hand-side",
        choices=("left", "right"),
        default=None,
        help="当前接入的 Revo2 是左手还是右手；默认跟随 --controller",
    )
    ap.add_argument("--hand-port", default=None, help="Revo2 飞线 USB-RS485 串口，例如 /dev/ttyUSB0；默认自动探测")
    ap.add_argument("--hand-baudrate", type=int, default=None, help="Revo2 波特率；指定串口时默认 460800")
    ap.add_argument("--hand-slave-id", type=lambda raw: int(raw, 0), default=None, help="Revo2 从站号；指定串口时默认 0x7e")
    return ap


async def main() -> None:
    args = build_arg_parser().parse_args()
    hand_side = args.hand_side or args.controller

    # 1) 先把 PICO 那头拉起来。SDK 未装时 XrClient 会降级成 dummy（全零），
    #    那种情况下不会崩，但手也不会动 —— 正好一眼就看出是上游没通。
    xr = XrClient()
    xr.init()
    logger.info("XrClient 已初始化。")

    # 2) 再开 Revo2（放后面是因为 PICO 初始化快、Revo2 探测可能等几秒）
    hand = Revo2HandDriver(
        Revo2HandConfig(
            side=hand_side,
            port=args.hand_port,
            baudrate=args.hand_baudrate,
            slave_id=args.hand_slave_id,
        )
    )
    try:
        await hand.connect()
        logger.info(
            "开始 PICO(%s) -> %s 实机遥操 @ %dHz。按 Ctrl+C 退出。",
            args.controller,
            hand.label,
            CTRL_HZ,
        )
        logger.info(
            "验收：trigger=食+中+无名+小 四指屈曲 / grip=拇指屈伸(ThumbFlex) / 拇指内外收(ThumbAux) 固定 %.0f%%",
            THUMB_OPPOSITION_DEFAULT * 100,
        )

        tick = 0
        while True:
            # 真实 PICO 输入（snapshot 同步返回，几十 us 级，不阻塞 asyncio loop）
            snap = xr.snapshot()
            trigger, grip = _pick_hand_inputs(snap, args.controller)

            targets = compute_revo2_targets(
                trigger=trigger,
                grip=grip,
                thumb_opposition=THUMB_OPPOSITION_DEFAULT,
            )
            positions = await hand.set_normalized_targets(targets)

            if tick % LOG_EVERY_N_TICKS == 0:
                logger.info(
                    "%s trig=%.2f grip=%.2f -> %s sdk=%s",
                    args.controller,
                    trigger,
                    grip,
                    hand.label,
                    positions,
                )

            tick += 1
            await asyncio.sleep(CTRL_DT)
    finally:
        # 清理顺序：先断 Revo2（释放串口），再关 XrClient
        try:
            await hand.close()
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
