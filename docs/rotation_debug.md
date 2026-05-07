先改这里，别动平移。

现在旋转在 [arm_teleop.py](/home/standard/pico_hand_arm_teleop/python_server/core/arm_teleop.py:150) 是这样：

```python
if self._track_rotation:
    R_delta = self._clutch.T_vr_init[:3, :3].T @ T_vr_now[:3, :3]
    T_cmd_raw[:3, :3] = self._clutch.T_ee_init[:3, :3] @ R_delta
```

这表示“手柄局部坐标系里的旋转增量，直接当成末端局部坐标系旋转”。问题是 PICO 手柄本身的局部轴，基本不可能刚好等于机械臂末端局部轴，所以会出现你说的：能转，但轴都不对。

第一版你改成这个：

```python
if self._track_rotation:
    R_vr_init = self._clutch.T_vr_init[:3, :3]
    R_vr_now = T_vr_now[:3, :3]
    R_ee_init = self._clutch.T_ee_init[:3, :3]

    R_delta = R_vr_now @ R_vr_init.T
    T_cmd_raw[:3, :3] = R_delta @ R_ee_init
```

意思变成：“在左臂控制坐标系里看手柄发生了什么世界系旋转，就把同样的世界系旋转作用到末端。”这个更符合你现在说的 `+X 前、+Y 上、+Z 左` 这种直觉。

测试命令用小一点：

```bash
./run.sh -m tools.test_xr_to_arm --mode pico --controller left \
  --hz 20 --vel 5 --acc 5 --workspace-margin-m 0.10 \
  --max-step-mm 4 --max-rot-deg 1 --ik-mode nsp --track-rotation
```

如果这样之后“轴对了但方向反”，再去改 [pico_streamer.py](/home/standard/pico_hand_arm_teleop/python_server/core/pico_streamer.py:115) 的 `rotation_basis`，不要动 `position_basis`：

```python
rotation_basis=LEFT_ARM_CONTROL_BASIS_FROM_LEGACY,
```

先可以试成：

```python
rotation_basis=np.diag([-1.0, 1.0, -1.0]),
```

但我建议第一步只改 `arm_teleop.py` 这个乘法顺序，因为你现在这个症状更像“局部轴/世界轴用错了”，不是简单符号翻反。