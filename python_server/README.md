# PICO Hand Arm Teleop

## 现场 Revo2 飞线方案

当前主线不再走天机末端 8Pin 485 透传，Revo2 直接用 USB-RS485 飞线接电脑。
已验证参数：

```text
port: /dev/ttyUSB*
baudrate: 460800
slave_id: 0x7E
```

临时打开串口权限：

```bash
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
```

长期权限方案：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

## 常用现场命令

在仓库根目录：

```bash
cd ~/pico_hand_arm_teleop/python_server
```

Revo2 最小开合：

```bash
./run.sh -m tools.test_hand_basic
```

PICO 控 Revo2，左手柄控左手：

```bash
./run.sh -m tools.test_xr_to_hand \
  --controller left \
  --hand-side left \
  --hand-port /dev/ttyUSB0 \
  --hand-baudrate 460800 \
  --hand-slave-id 0x7e
```

PICO 控 A 臂 + 左 Revo2：

```bash
export MARVIN_IP=192.168.71.190
./run.sh -m tools.test_xr_to_robot \
  --arm-controller left \
  --hand-controller left \
  --hand-side left \
  --hand-port /dev/ttyUSB0 \
  --hand-baudrate 460800 \
  --hand-slave-id 0x7e \
  --ik-mode nsp clutch \
  --hz 50 \
  --vel 50 \
  --acc 50 \
  --workspace-margin-m 0.05
```

如果 `/dev/ttyUSB0` 不是 Revo2，先用：

```bash
ls -l /dev/ttyUSB*
```

再把命令里的 `--hand-port` 改成对应设备。
