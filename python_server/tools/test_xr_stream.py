"""XRoboToolkit SDK 数据流测试脚本。

只读、不写任何硬件；用来验证：

1. ``xrobotoolkit_sdk`` 能在本机 import 并 init；
2. PICO 4 / 4 Ultra 戴上后，headset/右手柄的 pose、trigger、grip、button 能被读到；
3. 数据刷新正常（非全零、非恒定）。

用法：

    cd python_server
    uv run python -m tools.test_xr_stream
"""

from __future__ import annotations

import logging
import sys
import time
from pathlib import Path

_PKG_ROOT = Path(__file__).resolve().parent.parent
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from core.xr_client import XrClient


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger("test_xr_stream")


PRINT_HZ = 10  # 打印频率，够肉眼确认就行
PRINT_DT = 1.0 / PRINT_HZ


def _fmt_pose(p) -> str:
    return (
        f"pos=[{p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f}] "
        f"quat=[{p[3]:+.3f},{p[4]:+.3f},{p[5]:+.3f},{p[6]:+.3f}]"
    )


def main() -> None:
    xr = XrClient()
    xr.init()
    logger.info("XrClient 就绪。Ctrl+C 退出。")
    logger.info("操作建议：动一下 PICO 头显 / 右手柄 / 按 trigger / 按 grip / 按 A / 按菜单键。")

    try:
        while True:
            snap = xr.snapshot()
            logger.info(
                "head %s | ctrl %s | trig=%.2f grip=%.2f A=%s menu=%s",
                _fmt_pose(snap.headset_pose),
                _fmt_pose(snap.right_pose),
                snap.right_trigger,
                snap.right_grip,
                snap.button_a,
                snap.button_menu,
            )
            time.sleep(PRINT_DT)
    except KeyboardInterrupt:
        logger.info("用户中断。")
    finally:
        xr.close()


if __name__ == "__main__":
    main()
