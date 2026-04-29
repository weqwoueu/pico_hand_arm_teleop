"""Revo2 灵巧手最小测试脚本：自动识别端口，做开合循环。

用法：

    cd python_server
    uv run python -m tools.test_hand_basic

前置条件：

1. Revo2 已上电，USB-RS485 线接到主机；
2. 当前用户对 ``/dev/ttyUSB*`` 有读写权限（必要时 ``sudo usermod -aG dialout $USER``）；
3. 已安装 SDK：``uv add bc-stark-sdk colorlog``。
"""

from __future__ import annotations

import asyncio
import logging
import sys
import time

try:
    from bc_stark_sdk import main_mod as libstark  # type: ignore
except Exception as exc:  # noqa: BLE001
    print(f"[ERROR] 未安装 bc-stark-sdk: {exc}", file=sys.stderr)
    print("    请在 python_server 下执行: uv add bc-stark-sdk colorlog", file=sys.stderr)
    sys.exit(1)


logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
)
logger = logging.getLogger("test_hand_basic")


POS_OPEN = [0, 0, 0, 0, 0, 0]
POS_CLOSED = [400, 0, 1000, 1000, 1000, 1000]


async def _auto_open():
    """自动探测端口 + 波特率 + 从站 ID 并打开 Modbus 连接。"""
    logger.info("自动探测 Revo2 设备（可能需要几秒）...")
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
    client, slave_id = await _auto_open()
    try:
        info = await client.get_device_info(slave_id)
        logger.info("设备信息: %s", getattr(info, "description", info))

        # 归一化模式：位置量程变为 0 ~ 1000（0.1% 精度）
        await client.set_finger_unit_mode(slave_id, libstark.FingerUnitMode.Normalized)

        speeds = [1000] * 6  # 各关节最大速度
        logger.info("开始开合循环，按 Ctrl+C 退出。")

        cycle = 0
        while True:
            positions = POS_OPEN if cycle % 2 == 0 else POS_CLOSED
            logger.info("-> positions=%s", positions)
            await client.set_finger_positions_and_speeds(slave_id, positions, speeds)
            await asyncio.sleep(1.2)
            cycle += 1
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
