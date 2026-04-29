# third_party 使用说明

该目录用于存放第三方 SDK 与私有依赖。

## 建议结构

```text
third_party/
  xrobotoolkit_sdk/            # XR Robotics Toolkit（PICO 手柄数据源）
  stark-serialport-example/    # BrainCo Revo2 官方样例
  tianji_sdk/                  # 天机 Marvin M6CCS 机械臂 SDK
```

## 放置建议

1. 将 SDK 原始压缩包保留在组织内部制品库（不要直接提交到 Git）；
2. 解压后放到上述对应子目录；
3. 若包含 Python 模块，可在 `python_server/requirements.txt` 中使用本地路径，
   或执行 `uv pip install <path>`；
4. 若包含动态库（`.so`），请确保运行环境可找到库路径（例如设置 `LD_LIBRARY_PATH`）。

## 版本管理建议

- 记录 SDK 版本号、发布日期与变更日志；
- 若体积较大，推荐使用 Git LFS 或内部制品仓库管理。
