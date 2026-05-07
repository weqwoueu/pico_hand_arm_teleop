# 新电脑环境与联调手册

这份文档按“换一台电脑继续干活”的顺序写。目标不是只跑一个 demo，而是把整条路线接起来：

```text
PICO 头显 / 手柄
  -> XRoboToolkit PC Service
  -> xrobotoolkit_sdk
  -> python_server
  -> 天机 Marvin A 臂 arm-only / NSP 臂角控制
  -> Revo2 手
  -> PICO 摇操采集数据
  -> 后续训练
```

当前阶段重点是 **PICO 控 A 臂**。手臂测顺以后，再把 Revo2 手接回来，最后做同步数据采集。

## 0. 旧电脑推送前检查

在旧电脑仓库根目录：

```bash
git status --short --branch
```

确认要带走的改动都提交。当前 arm-only 相关核心改动在：

```text
python_server/tools/test_xr_to_arm.py
docs/setup_guide.md
```

如果 `third_party/TJ_FX_ROBOT_CONTRL_SDK/DEMO_PYTHON/showcase_position.py` 只是现场试动作留下的草稿，不想带到新电脑，就不要 `git add` 它。

推全部本地分支到 GitHub：

```bash
git push -u origin --all
git push origin --tags
```

以后某个分支已经绑定 upstream 后，在那个分支上直接：

```bash
git push
```

## 1. 新电脑拉仓库

```bash
cd ~
git clone https://github.com/weqwoueu/pico_hand_arm_teleop.git
cd pico_hand_arm_teleop
git branch -a
git switch 臂角控制优化
```

如果你想从别的分支继续：

```bash
git switch 手臂控制解耦
# 或
git switch master
```

## 2. Python 与 PICO pybind 环境

推荐 Ubuntu 22.04 / 24.04，Python 必须用 3.10。`xrobotoolkit_sdk` 是 `cpython-310` ABI，用 3.11/3.12 会 import 失败。

```bash
sudo apt update
sudo apt install -y git curl build-essential
```

项目里有安装脚本，作用和 GR00T-WholeBodyControl 的 `install_pico.sh` 类似：自动准备 uv、Python 3.10、`python_server/.venv`，并把 XRoboToolkit pybind 装进 venv。

```bash
cd ~/pico_hand_arm_teleop
bash scripts/install_pico_env.sh
```

脚本默认把 pybind clone 到：

```text
external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/
```

这个目录已被 `.gitignore` 忽略，不会被推到 GitHub。

基础自检：

```bash
cd ~/pico_hand_arm_teleop/python_server
./run.sh -m py_compile tools/test_xr_to_arm.py
./run.sh -c "import numpy, scipy; print('python deps ok')"
./run.sh -c "import xrobotoolkit_sdk as xrt; print(xrt.__file__)"
```

`python_server/.venv/` 不进 git，新电脑必须重新跑安装脚本或至少重新 `uv sync`。

## 3. XRoboToolkit 安装

PICO 数据链路需要两部分都齐：

```text
PICO APK
  -> /opt/apps/roboticsservice/RoboticsService
  -> libPXREARobotSDK.so
  -> xrobotoolkit_sdk.cpython-310-*.so
  -> python_server/core/xr_client.py
```

### 3.1 安装 PC Service

已验证版本是 v1.0.0 Ubuntu 22.04 amd64：

```bash
cd ~/Downloads
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb

sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo apt-get install -f
```

包名是 `roboticsservice`：

```bash
dpkg -l roboticsservice
dpkg -L roboticsservice | head
```

常用路径：

```text
/opt/apps/roboticsservice/runService.sh
/opt/apps/roboticsservice/run2D.sh
/opt/apps/roboticsservice/setting.ini
```

启动 Service：

```bash
/opt/apps/roboticsservice/runService.sh
pgrep -af RoboticsService | grep -v grep
```

正常会看到 `RoboticsService` 进程。端口一般是：

```bash
ss -tlnp | grep -E '60061|63901'
```

```text
127.0.0.1:60061  # Python pybind / gRPC
0.0.0.0:63901    # PICO APK 连接
```

### 3.2 准备 pybind `.so`

正常情况下 **不用手动准备**，第 2 节的脚本已经做了：

```bash
bash scripts/install_pico_env.sh
```

它会：

- clone `XRoboToolkit-PC-Service-Pybind`
- 安装 `cmake / pybind11 / setuptools`
- 设置 `CMAKE_PREFIX_PATH`
- `uv pip install --no-build-isolation -e ...`
- 生成 `python_server/.env.xr` 作为 `XRT_LIB_DIR` 兜底

如果脚本因为网络或官方仓库结构变化失败，再走手动兜底：从旧电脑拷贝已验证的 pybind 目录到新电脑这个位置：

```text
~/pico_hand_arm_teleop/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/
```

里面至少要有：

```text
xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so
lib/libPXREARobotSDK.so
```

然后再让脚本安装到 venv：

```bash
cd ~/pico_hand_arm_teleop
XRT_SKIP_PULL=1 bash scripts/install_pico_env.sh
```

验证 import：

```bash
cd ~/pico_hand_arm_teleop/python_server
./run.sh -c "import xrobotoolkit_sdk as xrt; print(xrt.__file__)"
```

如果这里报 `libPXREARobotSDK.so` 找不到，就是 `XRT_LIB_DIR` 没配对。

临时修复：

```bash
source ~/pico_hand_arm_teleop/python_server/.env.xr
```

### 3.3 验证 PICO 数据

1. 新电脑和 PICO 在同一局域网。
2. 电脑上启动 `/opt/apps/roboticsservice/runService.sh`。
3. PICO 里启动配套 APK，等 Service 日志出现 device connect。
4. 如数据全 0，打开 `/opt/apps/roboticsservice/run2D.sh`，确认 Tracking 相关项有勾选。

测试：

```bash
cd ~/pico_hand_arm_teleop/python_server
./run.sh -m tools.test_xr_stream
```

手柄动、trigger/grip/A/menu 变化时日志应该跟着变。

## 4. 天机 Marvin A 臂

天机 SDK 已放在仓库里，关键文件已经被 git 跟踪：

```text
third_party/TJ_FX_ROBOT_CONTRL_SDK/SDK_PYTHON/libKine.so
third_party/TJ_FX_ROBOT_CONTRL_SDK/SDK_PYTHON/libMarvinSDK.so
third_party/TJ_FX_ROBOT_CONTRL_SDK/DEMO_PYTHON/ccs_m6_31.MvKDCfg
```

新电脑一般不用额外安装天机 Python SDK，但要保证：

- 控制柜和电脑网线/网络通。
- `MARVIN_IP` 对。
- 没有旧进程占用 SDK 端口。
- 急停在手边。

设置 IP：

```bash
export MARVIN_IP=192.168.71.190
```

先跑 scripted 小幅自检：

```bash
cd ~/pico_hand_arm_teleop/python_server
./run.sh -m tools.test_xr_to_arm \
  --mode scripted \
  --axis auto \
  --duration 6 \
  --amp-mm 5 \
  --hz 20 \
  --vel 5 \
  --acc 5 \
  --workspace-margin-m 0.05 \
  --ik-mode nsp
```

再跑 PICO 控 A 臂：

```bash
./run.sh -m tools.test_xr_to_arm \
  --mode pico \
  --controller left \
  --hz 20 \
  --vel 5 \
  --acc 5 \
  --workspace-margin-m 0.10 \
  --max-step-mm 4 \
  --ik-mode nsp
```

当前约定：

- A 臂按左臂用，默认 `--controller left`。
- 离合键：左手柄 `X` 或左菜单键。
- 默认只跟随平移，末端姿态保持离合激活瞬间不变。
- `--track-rotation` 才会跟随手柄相对姿态。
- PICO 平移默认 `--xyz-scale=1,-1,1`，因为实测 Y 方向反了。

如果新电脑/新摆位发现 Y 又反了，临时切回：

```bash
--xyz-scale=1,1,1
```

如果 X/Z 也要镜像，用等号传负数，避免 argparse 把 `-1` 当选项：

```bash
--xyz-scale=-1,-1,1
```

### NSP 臂角控制说明

NSP 在这里按 **Null-Space Plane** 理解，也就是零空间臂角平面。

`test_xr_to_arm.py --ik-mode nsp` 的流程是：

1. 启动时读取当前 A 臂关节角。
2. 用 `fk_nsp()` 从当前姿态提取臂角平面。
3. 后续 IK 用 `ik_nsp()` 尽量保持这个臂角平面，让肘关节别乱翻。

所以启动 NSP 前，先把 A 臂摆到你满意的人形参考姿态。之前比较像人的参考姿态是：

```text
[40.6279, -60.2926, -89.4130, -70.8184, 0.0966, -20.0775, 0.9264] deg
```

不要在 J4 接近 0 度、肘关节近似伸直时启 NSP；SDK 文档也说参考姿态第四关节不能为零，否则臂角平面退化。

如果需要微调肘方向：

```bash
--ik-nsp-angle 15
# 或
--ik-nsp-angle -15
```

左臂方向按 SDK demo 注释：想让肘往上翘通常先试正角度；不对就反向。

## 5. Revo2 手

当前先测臂，手后面接回来。新电脑准备 Revo2 时：

```bash
cd ~/pico_hand_arm_teleop/python_server
uv sync
```

`bc-stark-sdk` 已在 `pyproject.toml` 里。插上 USB-RS485 后给串口权限：

```bash
ls /dev/ttyUSB*
sudo chmod 666 /dev/ttyUSB0
```

长期方案：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

纯手开合测试：

```bash
./run.sh -m tools.test_hand_basic
```

PICO -> Revo2 测试：

```bash
./run.sh -m tools.test_xr_to_hand
```

注意：`test_xr_to_hand.py` 当前不是 argparse 脚本，不要用 `--help`，它会真的开始连 PICO 和 Revo2。

## 6. 后续整体遥操与采数据

目标路线建议分三步：

1. **arm-only 稳定**：`tools.test_xr_to_arm --mode pico --ik-mode nsp` 能长时间稳定跑，方向、臂角、限位都可控。
2. **hand-only 稳定**：`tools.test_xr_to_hand` 能稳定用 trigger/grip 控 Revo2。
3. **arm + hand + record**：把 arm target、arm feedback、hand command、hand feedback、PICO pose/buttons 同步写成 episode。

采数据时至少记录这些字段：

```text
timestamp_ns
episode_id
head_pose
left_controller_pose
right_controller_pose
trigger / grip / buttons
arm_target_T
arm_feedback_joint
arm_feedback_T
hand_command_positions
hand_feedback_positions
mode / clutch_state / safety_flags
```

建议一开始同时存：

- raw PICO 数据
- 映射后的 arm/hand command
- 机器人实际 feedback

这样后面训练时既能学端到端，也能回放和修映射逻辑。

## 7. 常见问题

### `run.sh` 提示 `XRT_LIB_DIR 不存在`

新电脑上 pybind 目录位置和旧电脑不同。设置：

```bash
export XRT_LIB_DIR="$HOME/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib"
```

再跑：

```bash
./run.sh -c "import xrobotoolkit_sdk as xrt; print(xrt.__file__)"
```

### `xrobotoolkit_sdk` import 失败

检查三件事：

```bash
python --version
ls python_server/.venv/lib/python3.10/site-packages/xrobotoolkit_sdk*.so
echo "$XRT_LIB_DIR"
ls "$XRT_LIB_DIR/libPXREARobotSDK.so"
```

必须是 Python 3.10，对应 `cpython-310`。

### PICO 数据全 0

- PC Service 是否启动。
- PICO APK 是否连上这台电脑。
- 新电脑防火墙是否挡了 `63901`。
- `run2D.sh` 里 Tracking 相关项是否勾选。

### 天机报 `port bind failure` / 控制柜连接失败

常见原因是上一次 Ctrl+Z 把进程挂后台，没有释放 SDK 端口。

```bash
jobs -l
fg %1
# 回到前台后 Ctrl+C
```

如果找不到 job：

```bash
pgrep -af test_xr_to_arm
kill <pid>
```

然后重新跑。不要用 Ctrl+Z 停真机控制脚本，退出用 Ctrl+C。

### IK 经常失败或肘关节翻

- 起始姿态先摆成人形参考姿态，再启动 `--ik-mode nsp`。
- J4 不要接近 0 度。
- 工作空间先加 `--workspace-margin-m 0.05` 或 `0.10`。
- 速度加速度先用 `--vel 5 --acc 5`。
- 打开诊断：

```bash
--ik-debug
```

### Revo2 自动探测失败

- 确认 `/dev/ttyUSB*` 存在。
- 确认权限：`sudo chmod 666 /dev/ttyUSB0`。
- 确认手已上电、USB-RS485 接线正常。

## 8. 常用命令速查

```bash
# 拉仓库
git clone https://github.com/weqwoueu/pico_hand_arm_teleop.git
cd pico_hand_arm_teleop
git switch 臂角控制优化

# Python 环境
cd ~/pico_hand_arm_teleop
bash scripts/install_pico_env.sh
cd python_server

# PICO 数据
./run.sh -m tools.test_xr_stream

# A 臂 scripted
export MARVIN_IP=192.168.71.190
./run.sh -m tools.test_xr_to_arm --mode scripted --axis auto --duration 6 --amp-mm 5 --hz 20 --vel 5 --acc 5 --workspace-margin-m 0.05 --ik-mode nsp

# PICO 控 A 臂
./run.sh -m tools.test_xr_to_arm --mode pico --controller left --hz 20 --vel 5 --acc 5 --workspace-margin-m 0.10 --max-step-mm 4 --ik-mode nsp

# Revo2 手
./run.sh -m tools.test_hand_basic
./run.sh -m tools.test_xr_to_hand
```
