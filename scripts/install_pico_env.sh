#!/usr/bin/env bash
# Set up the Python 3.10 environment and XRoboToolkit pybind for PICO teleop.
#
# Run from anywhere:
#   bash scripts/install_pico_env.sh
#
# This intentionally does not install the XRoboToolkit PC Service .deb because
# that is a system package requiring sudo and differs by Ubuntu version.

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "$SCRIPT_DIR/.." && pwd)"
PY_DIR="$REPO_ROOT/python_server"

ARCH="$(uname -m)"
EXT_DIR="${XRT_EXTERNAL_DIR:-$REPO_ROOT/external_dependencies}"
XRT_DIR="${XRT_DIR:-$EXT_DIR/XRoboToolkit-PC-Service-Pybind_X86_and_ARM64}"
XRT_REPO="${XRT_REPO:-https://github.com/XR-Robotics/XRoboToolkit-PC-Service-Pybind.git}"

echo "[INFO] repo: $REPO_ROOT"
echo "[INFO] arch: $ARCH"

if ! command -v uv >/dev/null 2>&1; then
    echo "[INFO] uv not found; installing uv..."
    curl -LsSf https://astral.sh/uv/install.sh | sh
    if [[ -f "$HOME/.local/bin/env" ]]; then
        # shellcheck disable=SC1091
        source "$HOME/.local/bin/env"
    elif [[ -f "$HOME/.cargo/env" ]]; then
        # shellcheck disable=SC1091
        source "$HOME/.cargo/env"
    else
        export PATH="$HOME/.local/bin:$PATH"
    fi
fi

if ! command -v uv >/dev/null 2>&1; then
    echo "[ERROR] uv is still not on PATH. Add ~/.local/bin to PATH and rerun." >&2
    exit 1
fi

echo "[OK] $(uv --version)"

echo "[INFO] Installing/finding uv-managed Python 3.10..."
uv python install 3.10
MANAGED_PY="$(uv python find --no-project 3.10)"
echo "[OK] Python: $MANAGED_PY"

echo "[INFO] Creating/updating python_server/.venv..."
cd "$PY_DIR"
uv venv --python "$MANAGED_PY"
uv sync

echo "[INFO] Preparing XRoboToolkit pybind source..."
mkdir -p "$EXT_DIR"
if [[ ! -d "$XRT_DIR/.git" ]]; then
    if [[ -e "$XRT_DIR" ]]; then
        echo "[ERROR] $XRT_DIR exists but is not a git checkout." >&2
        echo "        Move it away or set XRT_DIR=/path/to/valid/pybind checkout." >&2
        exit 1
    fi
    git clone "$XRT_REPO" "$XRT_DIR"
else
    echo "[INFO] Existing pybind checkout found: $XRT_DIR"
    if [[ "${XRT_SKIP_PULL:-0}" != "1" ]]; then
        git -C "$XRT_DIR" pull --ff-only
    fi
fi

# Most x86_64 releases keep the runtime lib directly under lib/.
# Some arch-specific layouts use lib/<arch>/.
XRT_LIB_DIR_CANDIDATES=(
    "$XRT_DIR/lib"
    "$XRT_DIR/lib/$ARCH"
)

XRT_LIB_DIR_FOUND=""
for candidate in "${XRT_LIB_DIR_CANDIDATES[@]}"; do
    if [[ -f "$candidate/libPXREARobotSDK.so" ]]; then
        XRT_LIB_DIR_FOUND="$candidate"
        break
    fi
done

if [[ -z "$XRT_LIB_DIR_FOUND" ]]; then
    echo "[WARN] libPXREARobotSDK.so was not found under $XRT_DIR/lib." >&2
    echo "[WARN] If the pybind install fails, copy a known-good XRoboToolkit pybind bundle here:" >&2
    echo "       $XRT_DIR" >&2
    echo "[WARN] Expected one of:" >&2
    printf '       %s/libPXREARobotSDK.so\n' "${XRT_LIB_DIR_CANDIDATES[@]}" >&2
    XRT_LIB_DIR_FOUND="$XRT_DIR/lib"
fi

echo "[INFO] Installing XRoboToolkit SDK into .venv..."
# shellcheck disable=SC1091
source "$PY_DIR/.venv/bin/activate"
uv pip install cmake pybind11 setuptools wheel
export CMAKE_PREFIX_PATH="$(python -m pybind11 --cmakedir)"
echo "[OK] pybind11 cmake dir: $CMAKE_PREFIX_PATH"
uv pip install --no-build-isolation -e "$XRT_DIR"

echo "[INFO] Testing xrobotoolkit_sdk import..."
export XRT_LIB_DIR="$XRT_LIB_DIR_FOUND"
export LD_LIBRARY_PATH="$XRT_LIB_DIR${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
python - <<'PY'
import xrobotoolkit_sdk as xrt
print("[OK] xrobotoolkit_sdk:", xrt.__file__)
PY

cat > "$PY_DIR/.env.xr" <<EOF
# Source this only if python_server/run.sh cannot find libPXREARobotSDK.so.
export XRT_LIB_DIR="$XRT_LIB_DIR_FOUND"
EOF

cat <<EOF

══════════════════════════════════════════════════════════════
PICO Python environment is ready.

Next:
  1. Install/start XRoboToolkit PC Service if you have not:
       /opt/apps/roboticsservice/runService.sh

  2. Test PICO stream:
       cd "$PY_DIR"
       ./run.sh -m tools.test_xr_stream

If run.sh warns about XRT_LIB_DIR, run:
       source "$PY_DIR/.env.xr"
══════════════════════════════════════════════════════════════
EOF
