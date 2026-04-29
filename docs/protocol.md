# 数据流与坐标系约定

本系统不再使用 UDP/JSON 协议，所有数据在单进程内通过 Python 对象直接传递。
本文档定义从 PICO 到机械臂/灵巧手的**数据结构、坐标系与核心算法**。

## 1. 数据链路

```text
xrobotoolkit_sdk
       │   get_headset_pose / get_right_controller_pose / get_right_trigger / get_right_grip / get_X_button
       ▼
XrClient.snapshot() -> ControllerSnapshot
       ▼
PicoStreamer.step(T_ee_now) -> TeleopCommand
       ▼
TianjiRevoHardwareNode
       ├─ arm.servo_cartesian(T_target)      # 机械臂末端 4x4 齐次矩阵
       └─ hand.set_finger_positions(...)     # Revo2 五指归一化目标 [0, 1]
```

## 2. 坐标系

### 2.1 PICO (OpenXR 约定)

- 右手系，Y 轴向上；
- 原点为头显空间原点（由 runtime 决定）。

### 2.2 机器人基座系

- 右手系，Z 轴向上；
- X 轴指向机器人正前方，Y 轴向左。

### 2.3 基向量变换 `R_ROBOT_FROM_PICO`

```text
X_pico  ->  -Y_robot
Y_pico  ->  +Z_robot
Z_pico  ->  -X_robot
```

## 3. 位姿变换 `xr_pose_to_T(headset, ctrl)`

1. 将 headset / 右手柄的 7D pose 用 `R_ROBOT_FROM_PICO` 做基变换到机器人系；
2. 以 headset 位置为原点计算右手柄相对位移 `p_rel`；
3. 抽取 headset 当前朝向的 yaw，构造 `R_inv_yaw = Rz(yaw).T`；
4. `p_out = R_inv_yaw @ p_rel`、`R_out = R_inv_yaw @ R_ctrl`；
5. 拼装 4x4 齐次矩阵 `T_target_se3` 返回。

这样"操作者在现实空间原地转身"不会改变输出给机器人的指令。

## 4. 离合 (Clutch) 状态机

- **切换条件**：`A键` 或 `菜单键` 的上升沿。
- **激活瞬间**：锁存 `T_vr_init`（VR 侧处理后位姿）与 `T_ee_init`（机械臂末端实时位姿），并重置 EMA 滤波器。
- **激活期间**：
  ```text
  T_cmd = T_ee_init @ inv(T_vr_init) @ T_vr_now
        → EMA 滤波
        → WORKSPACE_LIMITS 钳位
  ```
- **非激活**：机械臂沿用当前位姿（不做增量）。

## 5. 滤波参数

- 平移：一阶低通 `x = α * x_new + (1-α) * x`，默认 `α = 0.25`；
- 旋转：四元数 Slerp，默认 `α = 0.20`；
- 参数位于 `python_server/core/pico_streamer.py` 顶部常量。

## 6. 工作空间限位

立方体 bounding box，仅对 `T_cmd` 的平移分量钳位：

| 轴 | 下限 (m) | 上限 (m) |
| :-- | :-- | :-- |
| X | 0.20 | 0.80 |
| Y | -0.50 | 0.50 |
| Z | 0.10 | 0.80 |

## 7. 灵巧手映射

- `right_trigger ∈ [0, 1]`  ->  食指；
- `right_grip    ∈ [0, 1]`  ->  中指、无名指、小指（同步）；
- 拇指：保持对掌位常量（默认 0.35）。

## 8. 主循环节拍

- 频率：100 Hz；
- 节拍控制使用 `time.perf_counter()` 基准累加，避免累计漂移。
