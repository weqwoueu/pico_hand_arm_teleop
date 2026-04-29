"""Revo2 灵巧手 + 映射函数联调脚本。

用模拟的 trigger / grip 走我们自己的 ``compute_revo2_targets`` + ``to_sdk_positions``，
实机验证 "trigger -> 食指"、"grip -> 中/无名/小指"、"拇指对掌" 是否符合预期。

用法：

    cd python_server
    uv run python -m tools.test_hand_mapping
"""

from __future__ import annotations

import asyncio
import logging
import math
import sys
import time
from pathlib import Path

# 兼容两种启动方式：
#   1) python -m tools.test_hand_mapping  （推荐）
#   2) python tools/test_hand_mapping.py  （直接跑单文件）
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


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger("test_hand_mapping")


# 控制频率。Revo2 官方示例默认 100Hz（10ms），过低会肉眼可见"台阶"。
# 经验值：
#   - 50 Hz 明显卡顿
#   - 100 Hz 刚好顺滑（官方默认）
#   - 200 Hz 更细腻，但串口带宽吃得更紧，Modbus 往返偶尔会丢拍
CTRL_HZ = 100
CTRL_DT = 1.0 / CTRL_HZ

# 模拟信号周期（秒）
TRIGGER_PERIOD = 3.0
GRIP_PERIOD = 4.0


def _simulated_trigger_grip(t: float) -> tuple[float, float]:
    """用两个相位不同的余弦波模拟 trigger / grip 输入，值域 [0, 1]。"""
    trigger = 0.5 - 0.5 * math.cos(2.0 * math.pi * t / TRIGGER_PERIOD)
    grip = 0.5 - 0.5 * math.cos(2.0 * math.pi * t / GRIP_PERIOD + math.pi / 3.0)
    return trigger, grip


async def _auto_open():
    logger.info("自动探测 Revo2 设备...")
    protocol, port_name, baudrate, slave_id = await libstark.auto_detect_modbus_revo2(
        None, True
    )
    assert protocol == libstark.StarkProtocolType.Modbus
    logger.info(
        "探测成功: port=%s baudrate=%s slave_id=0x%x", port_name, baudrate, slave_id
    )
    client = await libstark.modbus_open(port_name, baudrate)
    return client, slave_id


async def main() -> None:
    client, slave_id = await _auto_open()
    try:
        info = await client.get_device_info(slave_id)
        logger.info("设备信息: %s", getattr(info, "description", info))

        await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized)
        speeds = [1000] * 6

        logger.info(
            "开始遥操作映射模拟（trigger 周期 %.1fs, grip 周期 %.1fs），Ctrl+C 退出。",
            TRIGGER_PERIOD,
            GRIP_PERIOD,
        )

        t0 = time.perf_counter()
        log_every = 10  # 每 10 拍打印一次，避免刷屏
        tick = 0
        while True:
            t = time.perf_counter() - t0
            trigger, grip = _simulated_trigger_grip(t)

            targets = compute_revo2_targets(
                trigger=trigger,
                grip=grip,
                thumb_opposition=THUMB_OPPOSITION_DEFAULT,
            )
            positions = to_sdk_positions(targets)

            await client.set_finger_positions_and_speeds(slave_id, positions, speeds)

            if tick % log_every == 0:
                logger.info(
                    "t=%5.2fs trig=%.2f grip=%.2f -> sdk=%s",
                    t,
                    trigger,
                    grip,
                    positions,
                )
            tick += 1
            await asyncio.sleep(CTRL_DT)
    finally:
        try:
            libstark.modbus_close(client)
        except Exception:
            pass
        logger.info("已断开 Revo2。")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("用户中断。")
        sys.exit(0)
