#!/usr/bin/env bash

set -euo pipefail

# 创建目录
mkdir -p \
  python_server \
  python_server/core \
  third_party \
  docs

# 创建空占位文件
touch \
  python_server/.gitignore \
  python_server/README.md \
  python_server/main.py \
  python_server/pyproject.toml \
  python_server/.python-version \
  python_server/requirements.txt \
  python_server/core/__init__.py \
  python_server/core/xr_client.py \
  python_server/core/pico_streamer.py \
  python_server/core/hardware_node.py \
  python_server/core/mapping_utils.py \
  third_party/README.md \
  docs/setup_guide.md \
  docs/protocol.md

# 写入 pyproject.toml 默认内容
cat > python_server/pyproject.toml <<'EOF'
[project]
name = "pico-hand-arm-teleop"
version = "0.1.0"
description = "PICO 手柄遥操作天机 Marvin M6CCS + BrainCo Revo2 的 Python 控制系统"
readme = "README.md"
requires-python = ">=3.11"
dependencies = [
    "numpy>=1.24",
    "scipy>=1.10",
]
EOF

# 写入 .python-version
cat > python_server/.python-version <<'EOF'
3.11
EOF

# 写入 requirements.txt
cat > python_server/requirements.txt <<'EOF'
# Python 运行依赖。SDK（xrobotoolkit_sdk、Revo2、Tianji）请按 docs/setup_guide.md 单独安装。
numpy>=1.24
scipy>=1.10
EOF

echo "Repository scaffold created successfully."
