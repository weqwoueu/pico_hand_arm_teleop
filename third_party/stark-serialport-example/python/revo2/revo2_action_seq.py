"""
Revo2 动作序列示例

本示例演示如何使用 Revo2 灵巧手的动作序列能力，包括：
- 创建并配置自定义动作序列
- 使用多种控制模式
- 上传并存储动作序列
- 执行内置动作序列
- 执行和管理自定义动作序列
- 读取并校验动作序列数据

注意事项：
- Revo2 支持多种控制模式：位置+时间、位置+速度、电流、速度
- 当位置值为 65535 (0xFFFF) 时，表示该手指保持原角度
- 动作序列参数使用物理量（角度、速度、电流）
- 设备最多支持 6 组自定义动作序列
"""
# 中文说明：
# 这个示例的流程是：连接设备 -> 上传自定义动作序列 -> 读回校验 -> 执行一次动作序列 -> 等待 Ctrl+C 退出。
# 注意 run_action_sequence() 默认只触发一次，不会自动循环执行。

import asyncio
import sys
from utils import setup_shutdown_event
from revo2_utils import *

# Revo2 动作序列配置
# Revo2 支持更复杂的动作序列，包含多种控制模式和详细参数
# 中文：每个序列项代表一个“动作片段”，按 index 顺序依次执行。
#       mode=1 表示“位置+时间”控制；positions 的单位是角度，durations 是每个手指到位时间（毫秒）。
sample_action_sequences = [
    {
        "index": 0,                          # 动作片段索引，用于标识该片段在序列中的顺序
        "duration_ms": 1000,                 # 该动作片段总执行时长（毫秒）
        "mode": 2,                           # 控制模式：1=位置+时间，2=位置+速度，3=电流，4=速度
        "positions": [0, 0, 0, 0, 0, 0],    # 目标角度(°)：[拇指, 拇指辅指, 食指, 中指, 无名指, 小指]
        "durations": [100, 100, 100, 100, 100, 100],  # 各手指到达目标角度的时间（毫秒）
        "speeds": [100, 100, 100, 100, 100, 100],        # 各手指速度(°/s)，仅在 mode=2 时生效
        "currents": [0, 0, 0, 0, 0, 0],      # 各手指电流(mA)，仅在 mode=3 时生效
    },
    {
        "index": 1,
        "duration_ms": 2000,
        "mode": 2,                           # 位置+时间控制
        "positions": [0, 0, 90, 0, 0, 0],  # 握拳动作：各手指闭合到指定角度
        "durations": [200, 200, 200, 200, 200, 200],  # 拇指和食指慢闭合，其余手指快闭合
        "speeds": [100, 100, 200, 100, 100, 100],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 2,
        "duration_ms": 1000,
        "mode": 2,
        "positions": [0, 0, 0, 90, 0, 0],    # 拇指部分闭合，其余手指完全张开
        "durations": [500, 500, 100, 100, 500, 500],
        "speeds": [100, 100, 100, 100, 100, 100],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 3,
        "duration_ms": 2000,
        "mode": 2,
        "positions": [0, 0, 0, 0, 90, 0],  # 再次执行握拳动作
        "durations": [2000, 2000, 50, 100, 50, 100],
        "speeds": [190, 160, 1100, 1100, 60, 1010],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 4,
        "duration_ms": 2000,
        "mode": 2,
        "positions": [0, 0, 0, 0, 0, 90],  
        "durations": [100, 100, 200, 100, 100, 10],
        "speeds": [100, 100, 100, 100, 100, 50],
        "currents": [0, 0, 0, 0, 0, 0],
    },
    {
        "index": 5,                          # 动作片段索引，用于标识该片段在序列中的顺序
        "duration_ms": 1000,                 # 该动作片段总执行时长（毫秒）
        "mode": 2,                           # 控制模式：1=位置+时间，2=位置+速度，3=电流，4=速度
        "positions": [0, 0, 0, 0, 0, 0],    # 目标角度(°)：[拇指, 拇指辅指, 食指, 中指, 无名指, 小指]
        "durations": [100, 100, 100, 100, 100, 100],  # 各手指到达目标角度的时间（毫秒）
        "speeds": [100, 100, 100, 100, 100, 100],        # 各手指速度(°/s)，仅在 mode=2 时生效
        "currents": [0, 0, 0, 0, 0, 0],      # 各手指电流(mA)，仅在 mode=3 时生效
    },
        {
        "index": 6,                          # 动作片段索引，用于标识该片段在序列中的顺序
        "duration_ms": 1000,                 # 该动作片段总执行时长（毫秒）
        "mode": 2,                           # 控制模式：1=位置+时间，2=位置+速度，3=电流，4=速度
        "positions": [0, 0, 90, 90, 90, 90],    # 目标角度(°)：[拇指, 拇指辅指, 食指, 中指, 无名指, 小指]
        "durations": [100, 100, 10, 50, 100, 100],  # 各手指到达目标角度的时间（毫秒）
        "speeds": [100, 100, 200, 100, 90, 100],        # 各手指速度(°/s)，仅在 mode=2 时生效
        "currents": [0, 0, 0, 0, 0, 0],      # 各手指电流(mA)，仅在 mode=3 时生效
    },
    
]


def action_sequence_info_to_list(action):
    """
    将动作序列字典转换为列表格式

    把动作序列配置转换成 SDK 需要的平铺列表，用于上传到设备。
    Revo2 的动作序列格式比 Revo1 更复杂，包含更多控制参数。
    Args:
        action (dict): 动作序列配置字典，包含 index、duration_ms、mode、
                       positions、durations、speeds、currents

    Returns:
        list: 转换后的列表格式
              [index, duration_ms, mode, positions..., durations..., speeds..., currents...]
    """
    # 中文：SDK 传输接口要求平铺后的 list 格式，不能直接传 dict。
    return (
        [action["index"], action["duration_ms"], action["mode"]]
        + action["positions"]
        + action["durations"]
        + action["speeds"]
        + action["currents"]
    )


async def main():
    """
    主函数：初始化灵巧手并执行动作序列控制
    """
    # 设置关闭事件监听
    # 中文：用于监听 Ctrl+C，方便优雅退出程序。
    shutdown_event = setup_shutdown_event(logger)

    # 连接 Revo2 设备
    # 中文：port_name=None 时自动扫描串口并连接到检测到的 Revo2 设备。
    (client, slave_id) = await open_modbus_revo2(port_name=None) # 如有需要可替换为具体串口名；None 表示自动检测

    # 转换动作序列格式
    # 中文：将示例动作从 dict 转成 SDK 需要的 list。
    mapped_sequences = map(lambda action: action_sequence_info_to_list(action), sample_action_sequences)
    action_sequences = list(mapped_sequences)

    # 上传自定义动作序列到设备
    # 设备支持最多 6 组自定义动作（CustomGesture1 ~ CustomGesture6）
    custom_action_id = libstark.ActionSequenceId.CustomGesture1
    await client.transfer_action_sequence(slave_id, custom_action_id, action_sequences)
    logger.info(f"Custom action sequence uploaded to {custom_action_id}")

    # 读取并校验已上传的动作序列
    # 设备支持最多 6 组内置动作和最多 24 组自定义动作
    action_result: libstark.ActionSequenceItem = await client.get_action_sequence(slave_id, custom_action_id)
    logger.info(f"Action sequence: {action_result.description}")

    # 执行内置动作序列示例（可选）
    # logger.info("Executing built-in fist gesture...")
    # await client.run_action_sequence(slave_id, libstark.ActionSequenceId.DefaultGestureFist)
    # await asyncio.sleep(3)  # 等待内置动作执行完成

    # 执行自定义动作序列
    # 中文：这里只执行 1 次。你看到“开合两次后停住”就是因为 sample_action_sequences 里只有 4 段动作。
    # 如果想循环执行，需要在 while 循环里反复调用 run_action_sequence。
    logger.info("Executing custom action sequence...")
    await client.run_action_sequence(slave_id, custom_action_id)

    # logger.info("Clearing custom action sequence...")
    # await client.clear_action_sequence(slave_id, custom_action_id)
    # await client.run_action_sequence(slave_id, custom_action_id)

    # 等待关闭事件（Ctrl+C 或其他退出信号）
    # 中文：动作执行完后程序会挂起等待退出信号，不会自动退出。
    await shutdown_event.wait()

    # 释放资源
    libstark.modbus_close(client)
    logger.info("Modbus client closed")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("User interrupted")
        sys.exit(0)
    except Exception as e:
        logger.error(f"Error: {e}", exc_info=True)
        sys.exit(1)
