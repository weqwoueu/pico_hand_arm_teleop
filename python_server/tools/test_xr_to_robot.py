"""真实 PICO -> 天机 A 臂 + Revo2 单手集成验收脚本。

初版目标很明确：先测试一侧手柄同时控制同侧手臂和同侧手。

默认映射：

- 左手柄 pose -> A 臂 TCP 目标，X 或左菜单切换机械臂离合；
- 默认 hand-mode=gripper：左手柄 trigger -> Revo2 五指同步抓握；
- hand-mode=two-channel：左手柄 trigger -> 四指，grip -> 拇指屈伸；
- Revo2 拇指对掌/内外收保持 ``THUMB_OPPOSITION_DEFAULT``。

用法（在 ``python_server`` 目录）::

    sudo chmod 666 /dev/ttyUSB0
    export MARVIN_IP=192.168.71.190
    ./run.sh -m tools.test_xr_to_robot --ik-mode nsp clutch \
        --hand-port /dev/ttyUSB0 --hand-baudrate 460800 --hand-slave-id 0x7e \
        --hand-mode gripper \
        --track-rotation --hz 50 --vel 70 --acc 70 

Ctrl+C：下使能 A 臂、关闭 Revo2 串口、释放 XR SDK。
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import os
import signal
import sys
import time
from pathlib import Path
from typing import Optional

import numpy as np  # type: ignore[import-not-found]

# 兼容两种启动方式：
#   1) ./run.sh -m tools.test_xr_to_robot
#   2) python tools/test_xr_to_robot.py
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
    TianjiArmDriver,
    build_workspace_limits,
    default_tianji_m6_config_path,
    limit_pose_step,
    normalize_ik_mode,
)
from core.hand_core import Revo2HandConfig, Revo2HandDriver  # noqa: E402
from core.mapping_utils import (  # noqa: E402
    HAND_MODES,
    THUMB_OPPOSITION_DEFAULT,
    build_hand_control_sample,
    normalize_hand_mode,
)
from core.pico_streamer import T_to_pos_rpy  # noqa: E402
from core.xr_client import ControllerSnapshot, XrClient  # noqa: E402

logger = logging.getLogger("test_xr_to_robot")


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


def _parse_int_auto(raw: str) -> int:
    try:
        return int(raw, 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"无法解析整数: {raw}") from exc


def _pick_hand_inputs(snap: ControllerSnapshot, side: str) -> tuple[float, float]:
    if side == "left":
        return float(snap.left_trigger), float(snap.left_grip)
    if side == "right":
        return float(snap.right_trigger), float(snap.right_grip)
    raise ValueError("side 只能是 'left' 或 'right'")


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="PICO -> A 臂 + Revo2 单手集成验收")
    ap.add_argument("--dummy-arm", action="store_true", help="不连接控制柜，只跑 A 臂运动学 + 真 Revo2")
    ap.add_argument(
        "--ip",
        default=os.environ.get("MARVIN_IP", "192.168.71.190"),
        help="控制柜 IP，也可用 MARVIN_IP",
    )
    ap.add_argument(
        "--config",
        type=Path,
        default=None,
        help="MvKDCfg 路径，默认 TJ SDK DEMO_PYTHON/ccs_m6_31.MvKDCfg",
    )
    ap.add_argument("--hz", type=float, default=50.0, help="整机遥操主循环频率，默认 50")
    ap.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="运行时长秒数；0 表示一直运行直到 Ctrl+C",
    )
    ap.add_argument("--vel", type=int, default=10, help="A 臂速度百分比 1~100")
    ap.add_argument("--acc", type=int, default=10, help="A 臂加速度百分比 1~100")
    ap.add_argument(
        "--workspace-margin-m",
        type=float,
        default=0.0,
        help="首跑安全栏：>0 时把工作空间临时收紧到当前末端 ±margin (米)",
    )
    ap.add_argument(
        "--max-step-mm",
        type=float,
        default=8.0,
        help="A 臂单拍最大平移目标变化，0 表示不限制",
    )
    ap.add_argument(
        "--max-rot-deg",
        type=float,
        default=6.0,
        help="A 臂单拍最大姿态目标变化，0 表示不限制",
    )
    ap.add_argument(
        "--seed-joints",
        type=_parse_seed_joints,
        default=DEFAULT_SEED_JOINTS_DEG,
        help="dummy 或反馈不可用时的 A 臂初始关节角，逗号分隔 7 个度数",
    )
    ap.add_argument(
        "--arm-control-mode",
        choices=("cart-impedance", "joint-impedance", "position"),
        default="joint-impedance",
        help="A 臂底层控制柜模式；上层仍是 PICO->TCP target->IK->关节命令链路",
    )
    ap.add_argument(
        "--arm-controller",
        choices=("left", "right"),
        default="left",
        help="用于控制 A 臂位姿/离合的 PICO 手柄，默认 left",
    )
    ap.add_argument(
        "--hand-controller",
        choices=("left", "right"),
        default="left",
        help="用于控制 Revo2 trigger/grip 的 PICO 手柄，默认 left",
    )
    ap.add_argument(
        "--hand-side",
        choices=("left", "right"),
        default="left",
        help="当前接入的 Revo2 是左手还是右手，默认 left",
    )
    ap.add_argument("--hand-port", default=None, help="Revo2 飞线 USB-RS485 串口，例如 /dev/ttyUSB0；默认自动探测")
    ap.add_argument("--hand-baudrate", type=int, default=None, help="Revo2 波特率；指定串口时默认 460800")
    ap.add_argument("--hand-slave-id", type=_parse_int_auto, default=None, help="Revo2 从站号；指定串口时默认 left=0x7e, right=0x7f")
    ap.add_argument("--hand-speed", type=int, default=1000, help="Revo2 下发速度 0~1000，默认 1000")
    ap.add_argument(
        "--hand-mode",
        choices=HAND_MODES,
        default="gripper",
        help="手部控制模式：gripper=trigger 一维夹爪；two-channel=trigger 四指 + grip 拇指",
    )
    ap.add_argument(
        "--hand-release-on-close",
        action="store_true",
        help="退出时先张开 Revo2 再关闭串口；默认不主动张开",
    )
    ap.add_argument(
        "--thumb-opposition",
        type=float,
        default=THUMB_OPPOSITION_DEFAULT,
        help="拇指对掌/内外收归一化固定值，[0,1]；1.0 对应当前 ThumbAux 标定上限",
    )
    ap.add_argument(
        "--track-rotation",
        action="store_true",
        help="A 臂同时跟随手柄相对姿态；默认只跟随平移、保持末端初始姿态",
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
        help="PICO 平移增量逐轴缩放/镜像，格式 sx,sy,sz；若含负数请用等号",
    )
    ap.add_argument("--ik-debug", action="store_true", help="打印 IK 限位拒绝的调试信息")
    ap.add_argument(
        "--ik-mode",
        nargs="+",
        default=["normal"],
        metavar="IK",
        help="IK 模式：normal；nsp/fixed；nsp last；nsp clutch",
    )
    ap.add_argument(
        "--ik-nsp-angle",
        type=float,
        default=0.0,
        help="NSP 臂角微调角度，单位 deg",
    )
    return ap


async def main_async() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    ap = build_arg_parser()
    args = ap.parse_args()

    if args.hz <= 0:
        raise ValueError("--hz 必须 > 0")
    if not 0.0 <= args.thumb_opposition <= 1.0:
        raise ValueError("--thumb-opposition 必须在 [0,1]")
    hand_mode = normalize_hand_mode(args.hand_mode)

    try:
        ik_mode, ik_nsp_strategy = normalize_ik_mode(args.ik_mode)
    except ValueError as exc:
        ap.error(str(exc))

    arm = TianjiArmDriver(
        dummy=args.dummy_arm,
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
    hand = Revo2HandDriver(
        Revo2HandConfig(
            side=args.hand_side,
            port=args.hand_port,
            baudrate=args.hand_baudrate,
            slave_id=args.hand_slave_id,
            speed=args.hand_speed,
            release_on_close=args.hand_release_on_close,
        )
    )

    stop = {"flag": False}

    def _on_signal(signum, _frame):  # noqa: ANN001
        logger.info("收到信号 %s，准备退出...", signum)
        stop["flag"] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    xr: Optional[XrClient] = None
    arm_teleop: Optional[ArmTeleopController] = None
    last_cmd_T: Optional[np.ndarray] = None
    tick = 0
    arm_ok_cnt = 0
    hand_ok_cnt = 0
    prev_active = False

    try:
        arm.connect()
        last_cmd_T = arm.get_current_pose_m()
        limits = build_workspace_limits(args.workspace_margin_m, last_cmd_T)

        xr = XrClient()
        xr.init()
        arm_teleop = ArmTeleopController(
            xr,
            workspace_limits=limits,
            side=args.arm_controller,
            translation_scale=args.translation_scale,
            xyz_scale=args.xyz_scale,
            track_rotation=args.track_rotation,
        )
        await hand.connect()

        xyz0, rpy0 = T_to_pos_rpy(last_cmd_T)
        logger.info(
            "整机起点 xyz_m=[%+.3f %+.3f %+.3f] rpy_deg=[%+.1f %+.1f %+.1f]",
            xyz0[0],
            xyz0[1],
            xyz0[2],
            rpy0[0],
            rpy0[1],
            rpy0[2],
        )
        logger.info(
            "开始 PICO -> A 臂 + %s @ %.1fHz；arm_controller=%s hand_controller=%s "
            "hand_side=%s hand_mode=%s rotation=%s control=%s ik=%s。%s 切 A 臂离合，Ctrl+C 退出。",
            hand.label,
            args.hz,
            args.arm_controller,
            args.hand_controller,
            args.hand_side,
            hand_mode,
            "follow" if args.track_rotation else "hold",
            args.arm_control_mode,
            arm.ik_mode_label,
            "X 或左菜单" if args.arm_controller == "left" else "A 或右菜单",
        )

        dt = 1.0 / args.hz
        next_tick = time.perf_counter()
        t0 = time.perf_counter()
        max_step_m = max(0.0, args.max_step_mm) * 0.001

        while not stop["flag"]:
            if args.duration > 0.0 and time.perf_counter() - t0 >= args.duration:
                logger.info("达到 --duration %.1fs，准备退出。", args.duration)
                break

            assert arm_teleop is not None and last_cmd_T is not None
            T_now = arm.get_current_pose_m()
            cmd = arm_teleop.step(T_now)
            snap = arm_teleop.last_snap
            if snap is None:
                hand_trigger = 0.0
                hand_grip = 0.0
            else:
                hand_trigger, hand_grip = _pick_hand_inputs(snap, args.hand_controller)

            hand_sample = build_hand_control_sample(
                side=args.hand_side,
                trigger=hand_trigger,
                grip=hand_grip,
                thumb_opposition=args.thumb_opposition,
                mode=hand_mode,
            )
            hand_positions = await hand.set_normalized_targets(hand_sample.targets)
            hand_ok_cnt += 1

            if cmd.active and not prev_active:
                arm.capture_clutch_nsp_reference(arm.last_joints_deg)
            prev_active = cmd.active

            if cmd.active:
                T_limited = limit_pose_step(
                    cmd.T_target,
                    last_cmd_T,
                    max_step_m=max_step_m,
                    max_rot_deg=max(0.0, args.max_rot_deg),
                )
                if arm.servo_cartesian_m(T_limited):
                    arm_ok_cnt += 1
                    last_cmd_T = T_limited
            else:
                last_cmd_T = T_now

            if tick % max(1, int(args.hz)) == 0:
                xyz, rpy = T_to_pos_rpy(last_cmd_T)
                logger.info(
                    "tick=%d arm_active=%s hand_side=%s hand_mode=%s hand_trig=%.2f hand_grip=%.2f grasp=%.2f "
                    "xyz_m=[%+.3f %+.3f %+.3f] rpy_deg=[%+.1f %+.1f %+.1f] "
                    "hand_sdk=%s arm_ok=%d hand_ok=%d",
                    tick,
                    cmd.active,
                    hand_sample.side,
                    hand_sample.mode,
                    hand_sample.raw_trigger,
                    hand_sample.raw_grip,
                    hand_sample.grasp_scalar,
                    xyz[0],
                    xyz[1],
                    xyz[2],
                    rpy[0],
                    rpy[1],
                    rpy[2],
                    hand_positions,
                    arm_ok_cnt,
                    hand_ok_cnt,
                )

            tick += 1
            next_tick += dt
            sleep_for = next_tick - time.perf_counter()
            if sleep_for > 0:
                await asyncio.sleep(sleep_for)
            else:
                next_tick = time.perf_counter()
    finally:
        try:
            arm.disconnect()
        except Exception as exc:  # noqa: BLE001
            logger.warning("A 臂断开异常: %s", exc)
        try:
            await hand.close()
        except Exception as exc:  # noqa: BLE001
            logger.warning("Revo2 断开异常: %s", exc)
        if xr is not None:
            try:
                xr.close()
            except Exception as exc:  # noqa: BLE001
                logger.warning("XrClient 关闭异常: %s", exc)
        logger.info(
            "退出完成：arm_ok=%d hand_ok=%d tick=%d。",
            arm_ok_cnt,
            hand_ok_cnt,
            tick,
        )


if __name__ == "__main__":
    try:
        asyncio.run(main_async())
    except KeyboardInterrupt:
        logger.info("用户中断。")
        sys.exit(0)
