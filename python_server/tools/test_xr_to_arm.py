"""天机 Marvin A 臂纯测试脚本（不连接灵巧手）。

机械臂核心逻辑已收口到 ``core.arm_core``：

1. ``ArmTeleopController``：PICO 手柄 -> 机械臂目标 TCP 位姿；
2. ``ScriptedPoseSource``：无 PICO scripted 小幅轨迹；
3. ``TianjiArmDriver``：天机 A 臂连接、控制模式、FK/IK/NSP、关节命令下发。

用法（在 ``python_server`` 目录）::

    # ① 不接控制柜：纯 IK / 轨迹链路自检
    ./run.sh -m tools.test_xr_to_arm --mode scripted --dummy --duration 3

    # ② 真机 + scripted：当前末端附近三轴自动正弦，校 X/Y/Z 物理方向
    export MARVIN_IP=192.168.71.190
    ./run.sh -m tools.test_xr_to_arm --mode scripted --axis auto --hz 20 --duration 6 \
        --amp-mm 5 --vel 5 --acc 5 --workspace-margin-m 0.05

    # ③ 真机 + PICO：A 臂是左臂，默认用左手柄 (X 或左菜单切离合)
    ./run.sh -m tools.test_xr_to_arm --mode pico --ik-mode nsp clutch \
        --track-rotation --controller left --hz 50 --vel 50 --acc 50

操作：

- ``--mode scripted``：不需要 PICO，自动在当前末端位姿附近做小幅正弦；
- ``--mode pico``：用 ``--controller`` 选边：
  - ``left``  → 左手柄 pose / trigger / grip，**X 键** 或 **左菜单键** 切换离合；
  - ``right`` → 右手柄 pose / trigger / grip，**A 键** 或 **右菜单键** 切换离合；
- ``--arm-control-mode`` 只切换控制柜底层状态，上层 PICO -> TCP target -> IK -> joint cmd
  链路保持一致，方便对比 cart-impedance / joint-impedance / position；
- ``--ik-mode nsp last``：每帧用上一帧/当前反馈关节刷新参考臂角平面；
- ``--ik-mode nsp clutch``：每次离合激活时刷新并固定参考臂角平面；
- Ctrl+C：下使能 A 臂并释放 SDK 连接。

安全建议：

- 真机首跑务必带 ``--workspace-margin-m``；
- ``--vel`` / ``--acc`` 5~10 起步，跟手没问题再加；
- ``--max-step-mm`` / ``--max-rot-deg`` 是单拍跳变上限；
- 急停始终触手可达。
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np  # type: ignore[import-not-found]

# 兼容两种启动方式：
#   1) ./run.sh -m tools.test_xr_to_arm
#   2) python tools/test_xr_to_arm.py
_PKG_ROOT = Path(__file__).resolve().parent.parent
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from core.arm_core import (  # noqa: E402
    CART_IMPEDANCE_D,
    CART_IMPEDANCE_K,
    DEFAULT_SEED_JOINTS_DEG,
    JOINT_IMPEDANCE_D,
    JOINT_IMPEDANCE_K,
    ArmTeleopController,
    ScriptedPoseSource,
    TianjiArmDriver,
    build_workspace_limits,
    default_tianji_m6_config_path,
    limit_pose_step,
    normalize_ik_mode,
)
from core.pico_streamer import T_to_pos_rpy  # noqa: E402
from core.xr_client import XrClient  # noqa: E402

logger = logging.getLogger("test_xr_to_arm")


def _parse_seed_joints(raw: str) -> list[float]:
    vals = [float(x.strip()) for x in raw.split(",") if x.strip()]
    if len(vals) != 7:
        raise argparse.ArgumentTypeError("--seed-joints 需要 7 个逗号分隔的角度值")
    return vals


def _parse_vec3(raw: str) -> np.ndarray:
    vals = [float(x.strip()) for x in raw.split(",") if x.strip()]
    if len(vals) != 3:
        raise argparse.ArgumentTypeError("需要 3 个逗号分隔数值，例如 1,1,1")
    return np.asarray(vals, dtype=np.float64)


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="天机 Marvin A 臂纯测试（不接手）")
    ap.add_argument(
        "--mode",
        choices=("scripted", "pico"),
        default="scripted",
        help="scripted=无 PICO 自动小幅运动；pico=PICO 手柄遥操",
    )
    ap.add_argument("--dummy", action="store_true", help="不连接控制柜，只跑运动学 + IK")
    ap.add_argument(
        "--ip",
        default=os.environ.get("MARVIN_IP", "192.168.1.190"),
        help="控制柜 IP，也可用 MARVIN_IP",
    )
    ap.add_argument(
        "--config",
        type=Path,
        default=None,
        help="MvKDCfg 路径，默认 TJ SDK DEMO_PYTHON/ccs_m6_31.MvKDCfg",
    )
    ap.add_argument("--hz", type=float, default=20.0, help="遥操下发频率，建议先 20")
    ap.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="运行时长秒数；0 表示一直运行直到 Ctrl+C",
    )
    ap.add_argument("--vel", type=int, default=10, help="A 臂速度百分比 1~100")
    ap.add_argument("--acc", type=int, default=10, help="A 臂加速度百分比 1~100")
    ap.add_argument(
        "--axis",
        choices=("auto", "x", "y", "z", "-x", "-y", "-z"),
        default="auto",
        help=(
            "scripted 模式的正弦运动方向；可用 -x/-y/-z 做反向验证；Marvin 基座系 X=前后/Y=上下/Z=左右，"
            "auto 选当前工作空间余量最大的轴"
        ),
    )
    ap.add_argument("--amp-mm", type=float, default=5.0, help="scripted 模式正弦幅值 mm")
    ap.add_argument("--period", type=float, default=4.0, help="scripted 模式正弦周期秒")
    ap.add_argument(
        "--max-step-mm",
        type=float,
        default=8.0,
        help="单拍最大平移目标变化，0 表示不限制",
    )
    ap.add_argument(
        "--max-rot-deg",
        type=float,
        default=6.0,
        help="单拍最大姿态目标变化，0 表示不限制",
    )
    ap.add_argument(
        "--seed-joints",
        type=_parse_seed_joints,
        default=DEFAULT_SEED_JOINTS_DEG,
        help="dummy 或反馈不可用时的 A 臂初始关节角，逗号分隔 7 个度数",
    )
    ap.add_argument(
        "--workspace-margin-m",
        type=float,
        default=0.0,
        help=(
            "首跑安全栏：>0 时把 PICO/scripted 的工作空间临时收紧到当前末端 ±margin (米)，"
            "和 core 全局 WORKSPACE_LIMITS 取交集。建议真机首测 0.05 ~ 0.10。"
        ),
    )
    ap.add_argument(
        "--arm-control-mode",
        choices=("cart-impedance", "joint-impedance", "position"),
        default="cart-impedance",
        help=(
            "A 臂底层控制柜模式。cart-impedance=当前默认笛卡尔阻抗；"
            "joint-impedance=关节阻抗；position=位置模式。三者都沿用 PICO->TCP target->IK->关节命令链路。"
        ),
    )
    ap.add_argument(
        "--controller",
        choices=("left", "right"),
        default="left",
        help=(
            "PICO 模式驱动 A 臂使用的手柄。Marvin 的 A 臂是左臂，默认用左手柄；"
            "右手柄请传 right。离合键自动切换：left=X 或左菜单，right=A 或右菜单。"
        ),
    )
    ap.add_argument(
        "--track-rotation",
        action="store_true",
        help="PICO 模式下同时跟随手柄相对姿态；默认只跟随平移、保持末端初始姿态",
    )
    ap.add_argument(
        "--translation-scale",
        type=float,
        default=1.0,
        help="PICO 手柄平移增量整体缩放，默认 1.0",
    )
    ap.add_argument(
        "--xyz-scale",
        type=_parse_vec3,
        default=np.ones(3, dtype=np.float64),
        help=(
            "PICO 平移增量逐轴缩放/镜像，格式 sx,sy,sz。左右手基础坐标已在 core 中处理；"
            "若含负数请用等号，如 --xyz-scale=-1,1,1"
        ),
    )
    ap.add_argument(
        "--ik-debug",
        action="store_true",
        help="IK 被 soft-limit 拒绝时，每次都打印 ret_joint、运行限位和所有候选解的限位粗判",
    )
    ap.add_argument(
        "--ik-mode",
        nargs="+",
        default=["normal"],
        metavar="IK",
        help=(
            "IK 模式：normal；nsp/fixed=启动时固定参考臂角平面；"
            "nsp last=每帧用上一帧/当前反馈关节刷新参考平面；"
            "nsp clutch=每次离合激活时刷新并固定参考平面"
        ),
    )
    ap.add_argument(
        "--ik-nsp-angle",
        type=float,
        default=0.0,
        help="NSP 臂角微调角度，单位 deg；左臂方向可先试 +15 / -15",
    )
    return ap


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    ap = build_arg_parser()
    args = ap.parse_args()

    if args.hz <= 0:
        raise ValueError("--hz 必须 > 0")
    try:
        ik_mode, ik_nsp_strategy = normalize_ik_mode(args.ik_mode)
    except ValueError as exc:
        ap.error(str(exc))

    arm = TianjiArmDriver(
        dummy=args.dummy,
        robot_ip=args.ip,
        config_path=args.config if args.config is not None else default_tianji_m6_config_path(),
        vel_ratio=args.vel,
        acc_ratio=args.acc,
        arm_control_mode=args.arm_control_mode,
        cart_k=CART_IMPEDANCE_K,
        cart_d=CART_IMPEDANCE_D,
        joint_k=JOINT_IMPEDANCE_K,
        joint_d=JOINT_IMPEDANCE_D,
        seed_joints_deg=args.seed_joints,
        ik_debug=args.ik_debug,
        ik_mode=ik_mode,
        ik_nsp_strategy=ik_nsp_strategy,
        ik_nsp_angle_deg=args.ik_nsp_angle,
    )

    stop = {"flag": False}

    def _on_signal(signum, _frame):  # noqa: ANN001
        logger.info("收到信号 %s，准备退出...", signum)
        stop["flag"] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    arm.connect()
    last_cmd_T = arm.get_current_pose_m()
    limits = build_workspace_limits(args.workspace_margin_m, last_cmd_T)

    xr: Optional[XrClient] = None
    arm_teleop: Optional[ArmTeleopController] = None
    if args.mode == "pico":
        xr = XrClient()
        xr.init()
        arm_teleop = ArmTeleopController(
            xr,
            workspace_limits=limits,
            side=args.controller,
            translation_scale=args.translation_scale,
            xyz_scale=args.xyz_scale,
            track_rotation=args.track_rotation,
        )

    xyz0, rpy0 = T_to_pos_rpy(last_cmd_T)
    logger.info(
        "Marvin 基座系约定：X=前后 / Y=上下 / Z=左右横移；起点 xyz_m=[%+.3f %+.3f %+.3f] rpy_deg=[%+.1f %+.1f %+.1f]",
        xyz0[0], xyz0[1], xyz0[2], rpy0[0], rpy0[1], rpy0[2],
    )
    logger.info(
        "本次工作空间限位 (m): x=(%.3f,%.3f) y=(%.3f,%.3f) z=(%.3f,%.3f)%s",
        limits["x"][0], limits["x"][1],
        limits["y"][0], limits["y"][1],
        limits["z"][0], limits["z"][1],
        f"  margin=±{args.workspace_margin_m:.3f}m" if args.workspace_margin_m > 0 else "",
    )

    if args.mode == "pico":
        logger.info(
            "开始 PICO -> A 臂 arm-only 遥操 @ %.1fHz；controller=%s，rotation=%s，"
            "control=%s，scale=%.2f xyz_scale=[%+.2f %+.2f %+.2f]，ik=%s，"
            "离合默认关闭，%s 切换离合，Ctrl+C 退出。",
            args.hz,
            args.controller,
            "follow" if args.track_rotation else "hold",
            args.arm_control_mode,
            args.translation_scale,
            args.xyz_scale[0],
            args.xyz_scale[1],
            args.xyz_scale[2],
            arm.ik_mode_label,
            "X 或左菜单" if args.controller == "left" else "A 或右菜单",
        )
    else:
        logger.info(
            "开始无 PICO scripted A 臂测试 @ %.1fHz: axis=%s amp=%.1fmm period=%.1fs control=%s ik=%s。Ctrl+C 退出。",
            args.hz,
            args.axis,
            args.amp_mm,
            args.period,
            args.arm_control_mode,
            arm.ik_mode_label,
        )

    dt = 1.0 / args.hz
    next_tick = time.perf_counter()
    tick = 0
    ok_cnt = 0
    prev_active = False
    scripted_source = (
        ScriptedPoseSource(
            center_T=last_cmd_T,
            axis=args.axis,
            amp_mm=args.amp_mm,
            period_s=args.period,
            limits=limits,
        )
        if args.mode == "scripted"
        else None
    )
    max_step_m = max(0.0, args.max_step_mm) * 0.001

    t0 = time.perf_counter()
    try:
        while not stop["flag"]:
            if args.duration > 0.0 and time.perf_counter() - t0 >= args.duration:
                logger.info("达到 --duration %.1fs，准备退出。", args.duration)
                break

            elapsed_s = time.perf_counter() - t0
            T_now = arm.get_current_pose_m()

            if scripted_source is not None:
                active = True
                T_target = scripted_source.step(elapsed_s)
                trigger = 0.0
                grip = 0.0
            else:
                assert arm_teleop is not None
                cmd = arm_teleop.step(T_now)
                active = cmd.active
                T_target = cmd.T_target
                trigger = cmd.trigger
                grip = cmd.grip

            if active and not prev_active:
                arm.capture_clutch_nsp_reference(arm.last_joints_deg)
            prev_active = active

            if active:
                T_limited = limit_pose_step(
                    T_target,
                    last_cmd_T,
                    max_step_m=max_step_m,
                    max_rot_deg=max(0.0, args.max_rot_deg),
                )
                if arm.servo_cartesian_m(T_limited):
                    ok_cnt += 1
                    last_cmd_T = T_limited
            else:
                last_cmd_T = T_now

            if tick % max(1, int(args.hz)) == 0:
                xyz, rpy = T_to_pos_rpy(last_cmd_T)
                logger.info(
                    "tick=%d active=%s trig=%.2f grip=%.2f target_xyz_m=[%+.3f %+.3f %+.3f] rpy_deg=[%+.1f %+.1f %+.1f] ok=%d",
                    tick,
                    active,
                    trigger,
                    grip,
                    xyz[0],
                    xyz[1],
                    xyz[2],
                    rpy[0],
                    rpy[1],
                    rpy[2],
                    ok_cnt,
                )

            tick += 1
            next_tick += dt
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()
    finally:
        arm.disconnect()
        if xr is not None:
            xr.close()
        logger.info("退出完成：下发成功帧 %d / tick %d。", ok_cnt, tick)


if __name__ == "__main__":
    main()
