# 环境与启动说明

本系统直接在一台 Linux 主机上跑 Python 主循环，无需 Unity，也不再使用 UDP。
数据链路为：

```text
PICO 4 / PICO 4 Ultra (右手柄)
        │
        │  xrobotoolkit_sdk (XRoboToolkit)
        ▼
    XrClient  ->  PicoStreamer  ->  TianjiRevoHardwareNode
                                   ├─ Tianji Marvin M6CCS (7DoF)
                                   └─ BrainCo Revo2 灵巧手
```

## 1. 创建 Python 环境（uv 管理，**必须 Python 3.10**）

`xrobotoolkit_sdk` 预编译的 `.so` 是 `cpython-310`，用其他版本 import 时会直接报 ABI 不匹配。

```bash
cd python_server
uv venv --python 3.10
uv sync
```

> `.python-version` 已经钉死 3.10.x，正常情况下 `uv sync` 自己就会装对版本。

运行时需要给 Revo2 的串口开权限（每次插拔 USB 之后都要再来一遍，也可以做 udev 规则一劳永逸）：

```bash
sudo chmod 666 /dev/ttyUSB0
```

## 2. 安装 XRoboToolkit（PC Service + pybind 两侧都要）

PICO 数据链路由两段组成，**两段都要齐**：

```
PICO 头显客户端 APK ──(TCP/UDP 局域网)──► PC Service (系统级服务)
                                           ▲
                                           │ gRPC (localhost)
                                           ▼
                            libPXREARobotSDK.so (客户端动态库)
                                           ▲
                                           │ import
                                           ▼
                            xrobotoolkit_sdk.cpython-310-*.so (pybind)
```

### 2.1 PC Service（系统级服务进程，.deb 安装）

下载 Ubuntu 22.04 amd64 预编译包（Releases v1.0.0，~97 MB）：

```bash
# 直链（GitHub Releases）
cd ~/Downloads
wget https://github.com/XR-Robotics/XRoboToolkit-PC-Service/releases/download/v1.0.0/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb

sudo dpkg -i XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb
sudo apt-get install -f    # 如果缺依赖，这条会补上
```

注册的 dpkg 包名是 **`roboticsservice`**（不是 `xrobototoolkit-pc-service`，厂商 "Hainan Chuangjian Weilai"）。以后排查用：

```bash
dpkg -l roboticsservice                     # 查状态和版本
dpkg -L roboticsservice                     # 列出所有已装文件
sudo apt-mark hold roboticsservice          # 锁版本，防误升级
dpkg -S /opt/apps/roboticsservice/xxx       # 反查某文件归哪个包
```

安装后的布局（本机已验证）：

| 用途 | 路径 |
| --- | --- |
| 安装根 | `/opt/apps/roboticsservice/` |
| **Service 启动脚本** | `/opt/apps/roboticsservice/runService.sh` |
| 2D Qt 调试面板 | `/opt/apps/roboticsservice/run2D.sh` |
| 3D Unity 演示 | `/opt/apps/roboticsservice/run3D.sh` |
| 数据录制器 | `/opt/apps/roboticsservice/runRobotDataRecorder.sh` |
| SDK 头文件 | `/opt/apps/roboticsservice/SDK/include/PXREARobotSDK.h` |
| 配置文件 | `/opt/apps/roboticsservice/setting.ini` |

> 没有注册 systemd unit，**需要手动**开一个终端跑 `runService.sh`。注意：`runService.sh` 是个 **launcher**，执行完会立刻 return（把 Service 派生到后台），真正的进程名是 `RoboticsService`（不是 `roboticsservice`，大小写有差），想确认是否在跑：
>
> ```bash
> pgrep -af RoboticsService | grep -v grep
> sudo ss -tlnp | grep RoboticsService
> ```

Service 启动后会监听两个 TCP 端口（本机已验证）：

| 端口 | 监听地址 | 用途 |
| --- | --- | --- |
| `60061` | `127.0.0.1` 仅本机 | gRPC，给 pybind / 本机 SDK 客户端用（`xrobotoolkit_sdk.init()` 连的就是这里） |
| `63901` | `*` 所有网卡 | TCP，给 PICO 头显客户端 APK 连（局域网发现 + 对话） |

### 2.2 Pybind（Python 客户端绑定）

仓库：<https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind>

本机源码在 `~/GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/`。因为 `uv pip install .` 跑 CMake 构建时找不到 pybind11，这里**绕开 uv**，直接把编译好的 `.so` 手动拷贝进 venv：

```bash
SRC=~/GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64
DST=~/pico_hand_arm_teleop/python_server/.venv/lib/python3.10/site-packages
cp "$SRC/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so" "$DST/"
```

运行期这个 `.so` 还依赖 `libPXREARobotSDK.so`（客户端动态库，跟着 pybind 仓库一起的）：

```
~/GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib/libPXREARobotSDK.so
```

### 2.3 `python_server/run.sh`：所有 Python 命令的统一入口

**不要直接 `python -m tools.xxx`**，会因为 `LD_LIBRARY_PATH` 找不到 `libPXREARobotSDK.so` 报错。`run.sh` 替你做了三件事：

- 注入 `LD_LIBRARY_PATH` 指向 pybind 仓库的 `lib/`
- 强制用 `.venv/bin/python`，不沾系统 python
- 自动 `cd` 到 `python_server/`，让 `-m tools.xxx` / `-m main` 的相对导入正常工作

用法：

```bash
cd ~/pico_hand_arm_teleop/python_server
./run.sh -m tools.test_xr_stream     # 10Hz 打印 PICO 数据
./run.sh -m tools.test_hand_basic    # Revo2 开合验证
./run.sh -m tools.test_hand_mapping  # Revo2 + 映射管线
./run.sh -m main                     # 100Hz 主循环
```

> 自定义 SDK lib 目录：`XRT_LIB_DIR=/some/other/lib ./run.sh ...`

### 2.4 完整联调顺序（实机验证时）

1. 终端 A：`/opt/apps/roboticsservice/runService.sh`（前台阻塞，盯日志）
2. PICO 头显：启动配套 Unity 客户端 APK，确认与 PC 同局域网，等 Service 日志出现 "device find" / "device connect"
3. （可选）终端 B：`/opt/apps/roboticsservice/run2D.sh` 打开 Qt 面板，**勾选 Tracking 子类**（Head/Controller/Hand 至少勾前两个），否则 Pybind 那边拿到的一直是 0
4. 终端 C：`cd ~/pico_hand_arm_teleop/python_server && ./run.sh -m tools.test_xr_stream`，应看到 pose/trigger/grip 随手柄实时变化

## 3. 安装 Revo2 SDK（可选：当前 Dummy 驱动无需）

示例代码位于 `third_party/stark-serialport-example/python/revo2/`，按其 README 安装即可：

```bash
cd third_party/stark-serialport-example/python
uv pip install -r requirements.txt
```

## 4. 安装天机机械臂 SDK

按厂方 SDK 文档连接 Tianji Marvin M6CCS（网口/笛卡尔伺服）。在
`python_server/core/hardware_node.py` 的 `TianjiArmDriver` 内把 `TODO` 替换为真实调用。

## 5. 启动服务

前提：PC Service 已在另一个终端 `runService.sh` 跑着，PICO 头显已连上（见 2.4）。

```bash
cd python_server
./run.sh -m main
```

（别用 `uv run python main.py`，会丢 `LD_LIBRARY_PATH`；`run.sh` 封装了所有环境。）

启动后效果：

- 戴上 PICO，右手柄握持；
- 按下右手柄 **A 键** 或 **菜单键** 激活离合（再次按下取消）；
- 激活后，手柄移动会映射为末端笛卡尔增量，机械臂跟随；
- `right_trigger` 控制食指弯曲；`right_grip` 控制中指/无名指/小指同步弯曲。

## 6. 常见问题

- `xrobotoolkit_sdk` 未安装：程序会以 dummy 模式运行，所有位姿返回默认值，便于上层调试。
- 机械臂不动：确认已按下 A/菜单键进入激活态，并检查日志 `离合激活` 提示。
- 控制抖动：调小 `python_server/core/pico_streamer.py` 中的 `EMA_ALPHA_*`。
- 超出工作空间：查看 `WORKSPACE_LIMITS` 是否与你机器人的布置一致。

## 7. 目录结构（Python 侧）

```text
python_server/
  run.sh                  # 环境封装：注入 LD_LIBRARY_PATH + .venv/bin/python
  main.py                 # 100Hz 主循环
  pyproject.toml
  uv.lock
  .python-version         # 3.10
  .venv/                  # uv 管的虚拟环境（名: pico-hand-arm-teleop）
  core/
    __init__.py
    xr_client.py          # XRoboToolkit SDK 安全封装
    pico_streamer.py      # 位姿变换 / 离合 / EMA 滤波 / 工作空间限位
    hardware_node.py      # Tianji + Revo2 Dummy 驱动
    mapping_utils.py      # Revo2 手指映射
  tools/                  # 独立测试脚本（不跑主循环）
    test_hand_basic.py    # 纯 SDK 开合循环
    test_hand_mapping.py  # 映射管线 + 模拟 trigger/grip 驱手 (100Hz)
    test_xr_stream.py     # 只读 PICO 数据, 10Hz 打印
```

## 8. 工作日志与下次接入点

### 2026-04-20 进展

**完成**
- [x] PC Service `.deb` 安装到 `/opt/apps/roboticsservice/`（v1.0.0 ubuntu-22.04-amd64）
- [x] pybind `.so`（cp310）手动 cp 进 `.venv/site-packages/`，`import xrobotoolkit_sdk` OK
- [x] 写 `python_server/run.sh` 封装 `LD_LIBRARY_PATH`，烟测通过
- [x] 干跑 `./run.sh -m tools.test_xr_stream`：`xrt.init()` 成功、10Hz 主循环不崩、pose 全 0（因为 Service 还没启）
- [x] `xr_client.py` 里的 SDK 兼容命名全对上了 `xrt` 实际属性（`init/close/get_A_button/get_right_menu_button/...`）

### 2026-04-21 进展

**完成**
- [x] ② 实机验证：PC Service 起、PICO APK 连上、`./run.sh -m tools.test_xr_stream` 看到 head/ctrl pose 实时变化，trigger/grip/A/menu 全部响应
- [x] 确认 Service 监听两个端口：`127.0.0.1:60061` (gRPC 给 pybind) + `*:63901` (给 PICO APK)
- [x] 确认 `.deb` 装的 `libPXREARobotSDK.so` 和 pybind 用的那份 **sha256 一致**，不存在版本分叉
- [x] 确认 `.deb` 真实包名是 `roboticsservice`（海南创见未来）

**下次接上的第一步**
1. 先做 ③ 之前建议：跑一次 `./run.sh -m main`（Dummy 手 + Dummy 臂 + 真实 PICO），确认 `PicoStreamer` 的坐标变换/去偏航/离合/EMA/workspace clamp 对真实数据都没崩——这是真实数据第一次过整条业务管线，有必要先 sanity check
2. 然后读 `third_party/stark-serialport-example/python/revo2/revo2_ctrl_right.py` + `revo2_utils.py`，决定 Revo2HandDriver 接法（后台 asyncio 线程 or `asyncio.run` 包装）
3. 写真实 `Revo2HandDriver`，替换 `hardware_node.py` 里的 Dummy

**随后（顺序不硬性）**
- [ ] ④ 拿到天机 SDK 后替换 `TianjiArmDriver`
- [ ] ⑤ 完整联调 `./run.sh -m main`（真实 PICO + 真实 Revo2 + 真实天机）

**尚未处理的小坑**
- `python_server/.git` 是 `uv init` 留下的子仓库，需要 `rm -rf python_server/.git` 然后根目录 `git init`——下次动手前先确认一次
- `test_xr_stream.py` 没装 signal handler，SIGTERM 杀不干净（今天被沙盒以 root 起过一次进程，kill 不动需要 `sudo kill -9`）；之后给它加个 `--duration` 参数 + signal handler 会更顺手
- `~/.bashrc` 里其实**没有** `LD_LIBRARY_PATH` 的导出（之前笔记说"已写进"不对），而且 `.bashrc` 非交互 shell 直接 return 也写不进去有效——已决定用 `run.sh` 项目本地方案，全局不动

**关键路径速查**

| 名称 | 路径 |
| --- | --- |
| pybind 源码 / `libPXREARobotSDK.so` 目录 | `~/GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/` |
| pybind `.so` 目标位置 | `~/pico_hand_arm_teleop/python_server/.venv/lib/python3.10/site-packages/xrobotoolkit_sdk.cpython-310-x86_64-linux-gnu.so` |
| PC Service Start | `/opt/apps/roboticsservice/runService.sh` |
| PC Service 2D 面板 | `/opt/apps/roboticsservice/run2D.sh` |
| PC Service 配置 | `/opt/apps/roboticsservice/setting.ini` |
| 项目统一入口 | `~/pico_hand_arm_teleop/python_server/run.sh` |
| 下载的 `.deb` 存档 | `~/Downloads/XRoboToolkit_PC_Service_1.0.0_ubuntu_22.04_amd64.deb` |

