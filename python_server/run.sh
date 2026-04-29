#!/usr/bin/env bash
# 项目本地运行封装脚本。
#
# 作用：
#   1. 给 xrobotoolkit_sdk (.so) 加载时需要的 libPXREARobotSDK.so 注入 LD_LIBRARY_PATH；
#   2. 固定用 ./.venv/bin/python 跑，避免碰到系统 python；
#   3. 把工作目录切到 python_server/，方便 `-m tools.xxx` / `-m main` 这种相对导入。
#
# 用法：
#   ./run.sh -m tools.test_xr_stream
#   ./run.sh -m tools.test_hand_basic
#   ./run.sh -m main
#   ./run.sh -c "import xrobotoolkit_sdk as xrt; print(xrt.__file__)"
#
# 想覆盖默认的 XR SDK lib 路径，导出 XRT_LIB_DIR 即可：
#   XRT_LIB_DIR=/some/other/lib ./run.sh -m tools.test_xr_stream

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

DEFAULT_XRT_LIB_DIR="$HOME/GR00T-WholeBodyControl/external_dependencies/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64/lib"
XRT_LIB_DIR="${XRT_LIB_DIR:-$DEFAULT_XRT_LIB_DIR}"

if [[ ! -d "$XRT_LIB_DIR" ]]; then
    echo "[run.sh][WARN] XRT_LIB_DIR 不存在: $XRT_LIB_DIR" >&2
    echo "[run.sh][WARN] xrobotoolkit_sdk 加载可能会找不到 libPXREARobotSDK.so" >&2
fi

export LD_LIBRARY_PATH="${XRT_LIB_DIR}${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

VENV_PY="$SCRIPT_DIR/.venv/bin/python"
if [[ ! -x "$VENV_PY" ]]; then
    echo "[run.sh][ERROR] 找不到 venv python: $VENV_PY" >&2
    echo "[run.sh][ERROR] 请先在 $SCRIPT_DIR 里 uv sync 创建 .venv" >&2
    exit 1
fi

cd "$SCRIPT_DIR"

if [[ $# -eq 0 ]]; then
    cat <<EOF
用法:
  $0 -m tools.test_xr_stream    # 只读 PICO 数据流
  $0 -m tools.test_hand_basic   # Revo2 开合验证
  $0 -m tools.test_hand_mapping # Revo2 映射管线
  $0 -m main                    # 100Hz 主循环
  $0 -c "import xrobotoolkit_sdk; print('ok')"

环境变量:
  XRT_LIB_DIR  自定义 libPXREARobotSDK.so 目录 (默认 $DEFAULT_XRT_LIB_DIR)
EOF
    exit 0
fi

exec "$VENV_PY" "$@"
