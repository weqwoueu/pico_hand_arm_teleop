"""天机 Marvin A 臂纯测试脚本（不连接灵巧手）。

机械臂侧只按 ``third_party/TJ_FX_ROBOT_CONTRL_SDK`` 官方 SDK 接口接入：

1. ``fx_robot.Marvin_Robot`` 负责连接、清错、订阅、切模式、下发关节命令；
2. ``fx_kine.Marvin_Kine`` 负责 FK/IK，PICO 笛卡尔目标先转 mm 再求 IK；
3. 轨迹层复用 ``core.pico_streamer`` 的滤波、限位和位姿打印工具。

用法（在 ``python_server`` 目录）::

    # ① 不接控制柜：纯 IK / 轨迹链路自检
    ./run.sh -m tools.test_xr_to_arm --mode scripted --dummy --duration 3

    # ② 真机 + scripted：当前末端附近三轴自动正弦，校 X/Y/Z 物理方向
    export MARVIN_IP=192.168.71.190
    ./run.sh -m tools.test_xr_to_arm --mode scripted --axis auto --hz 20 --duration 6 \
        --amp-mm 5 --vel 5 --acc 5 --workspace-margin-m 0.05

    # ③ 真机 + PICO：A 臂是左臂，默认用左手柄 (X 或左菜单切离合)
    ./run.sh -m tools.test_xr_to_arm --mode pico --controller left --hz 20 --vel 5 --acc 5 \
        --workspace-margin-m 0.10

    # 想用右手柄驱动 A 臂（不推荐）：
    ./run.sh -m tools.test_xr_to_arm --mode pico --controller right ...

操作：

- ``--mode scripted``：不需要 PICO，自动在当前末端位姿附近做小幅正弦；
- ``--mode pico``：用 ``--controller`` 选边：
  - ``left``  → 左手柄 pose / trigger / grip，**X 键** 或 **左菜单键** 切换离合；
  - ``right`` → 右手柄 pose / trigger / grip，**A 键** 或 **右菜单键** 切换离合；
- idle：只读 PICO 和臂状态，不下发；
- active：末端在当前位姿附近按选定手柄的相对增量跟随；
- Ctrl+C：下使能 A 臂并释放 SDK 连接。

安全建议：

- 真机首跑务必带 ``--workspace-margin-m``，限位会以 “当前末端 ± margin” 与全局
  ``WORKSPACE_LIMITS`` 取交集，越界目标会被 clamp 而不是直接奔出去；
- ``--vel`` / ``--acc`` 5~10 起步，跟手没问题再加；
- ``--max-step-mm`` / ``--max-rot-deg`` 是单拍跳变上限，丢帧或离合误触不会让臂炸；
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
from scipy.spatial.transform import Rotation as R  # type: ignore[import-not-found]

# 兼容两种启动方式：
#   1) ./run.sh -m tools.test_xr_to_arm
#   2) python tools/test_xr_to_arm.py
_PKG_ROOT = Path(__file__).resolve().parent.parent
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from core.pico_streamer import (  # noqa: E402
    PoseEmaFilter,
    PicoStreamer,
    T_to_pos_rpy,
    WORKSPACE_LIMITS,
    clamp_workspace,
)
from core.xr_client import XrClient  # noqa: E402

logger = logging.getLogger("test_xr_to_arm")

# SDK demo 中常用的 A 臂演示构型；dummy 模式和无反馈兜底使用。
DEFAULT_SEED_JOINTS_DEG = [21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68]


def _repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def _tj_sdk_root() -> Path:
    return _repo_root() / "third_party" / "TJ_FX_ROBOT_CONTRL_SDK"


def _ensure_tj_sdk_path() -> Path:
    root = _tj_sdk_root()
    if not root.is_dir():
        raise FileNotFoundError(f"未找到 TJ SDK 目录: {root}")
    root_s = str(root)
    if root_s not in sys.path:
        sys.path.insert(0, root_s)
    return root


def _default_cfg() -> Path:
    return _tj_sdk_root() / "DEMO_PYTHON" / "ccs_m6_31.MvKDCfg"


def _matrix4_to_rows(T: np.ndarray) -> list[list[float]]:
    if T.shape != (4, 4):
        raise ValueError(f"T shape must be (4, 4), got {T.shape}")
    return [[float(T[i, j]) for j in range(4)] for i in range(4)]


def _sdk_T_to_m(T_sdk: np.ndarray) -> np.ndarray:
    """SDK FK/IK 使用 mm 平移；PicoStreamer 使用 m 平移。"""
    T_m = T_sdk.copy()
    T_m[:3, 3] *= 0.001
    return T_m


def _m_T_to_sdk(T_m: np.ndarray) -> np.ndarray:
    T_sdk = T_m.copy()
    T_sdk[:3, 3] *= 1000.0
    return T_sdk


def _parse_seed_joints(raw: str) -> list[float]:
    vals = [float(x.strip()) for x in raw.split(",") if x.strip()]
    if len(vals) != 7:
        raise argparse.ArgumentTypeError("--seed-joints 需要 7 个逗号分隔的角度值")
    return vals


def _limit_pose_step(
    T_target: np.ndarray,
    T_last: np.ndarray,
    *,
    max_step_m: float,
    max_rot_deg: float,
) -> np.ndarray:
    """限制单拍目标跳变，防 PICO 丢帧或离合误触造成大步长。"""
    T_out = T_target.copy()

    delta_p = T_target[:3, 3] - T_last[:3, 3]
    dist = float(np.linalg.norm(delta_p))
    if max_step_m > 0.0 and dist > max_step_m:
        T_out[:3, 3] = T_last[:3, 3] + delta_p * (max_step_m / dist)

    if max_rot_deg > 0.0:
        r_last = R.from_matrix(T_last[:3, :3])
        r_target = R.from_matrix(T_target[:3, :3])
        r_delta = r_last.inv() * r_target
        rotvec = r_delta.as_rotvec()
        angle = float(np.linalg.norm(rotvec))
        max_angle = np.deg2rad(max_rot_deg)
        if angle > max_angle and angle > 1e-9:
            r_limited = r_last * R.from_rotvec(rotvec * (max_angle / angle))
            T_out[:3, :3] = r_limited.as_matrix()

    return T_out


def _build_workspace_limits(
    margin_m: float, center_T: np.ndarray
) -> dict:
    """根据当前末端位姿生成一个临时的 bounding box。

    - ``margin_m <= 0``：直接复用 ``core.pico_streamer.WORKSPACE_LIMITS``；
    - 否则：以当前末端 xyz 为中心，每个轴 ±margin_m 米，并与全局 limits 取交集，
      避免 margin 把全局保护写穿。
    """
    if margin_m <= 0.0:
        return dict(WORKSPACE_LIMITS)
    p = center_T[:3, 3]
    out: dict = {}
    for i, name in enumerate(("x", "y", "z")):
        g_lo, g_hi = WORKSPACE_LIMITS[name]
        lo = max(g_lo, float(p[i] - margin_m))
        hi = min(g_hi, float(p[i] + margin_m))
        if lo >= hi:
            raise RuntimeError(
                f"workspace margin 计算失败：轴 {name} 当前 {p[i]:.3f} 在全局 limits "
                f"({g_lo:.3f},{g_hi:.3f}) 之外，请检查 FK 起点或先收回安全姿态"
            )
        out[name] = (lo, hi)
    return out


class ScriptedPoseSource:
    """无 PICO 测试输入源：复用 core 的滤波/限位，在当前位姿附近生成小幅正弦目标。"""

    def __init__(
        self,
        *,
        center_T: np.ndarray,
        axis: str,
        amp_mm: float,
        period_s: float,
        limits: dict,
    ) -> None:
        if axis not in {"auto", "x", "y", "z"}:
            raise ValueError("--axis 只能是 auto/x/y/z")
        if period_s <= 0.0:
            raise ValueError("--period 必须 > 0")

        self._limits = limits
        self._center_T = clamp_workspace(center_T, limits=limits)
        chosen_axis = (
            self._choose_axis(self._center_T, limits) if axis == "auto" else axis
        )
        self._axis_idx = {"x": 0, "y": 1, "z": 2}[chosen_axis]
        self._amp_m = max(0.0, amp_mm) * 0.001
        self._period_s = period_s
        self._filter = PoseEmaFilter()
        self._filter.reset(self._center_T)

        raw_xyz = center_T[:3, 3]
        center_xyz = self._center_T[:3, 3]
        if not np.allclose(raw_xyz, center_xyz, atol=1e-6):
            logger.warning(
                "当前末端超出/贴近 scripted 工作空间，中心已限位: raw=[%.3f %.3f %.3f] -> center=[%.3f %.3f %.3f]",
                raw_xyz[0],
                raw_xyz[1],
                raw_xyz[2],
                center_xyz[0],
                center_xyz[1],
                center_xyz[2],
            )
        logger.info("scripted 轨迹轴: %s", chosen_axis)

    def step(self, elapsed_s: float) -> np.ndarray:
        T = self._center_T.copy()
        phase = 2.0 * np.pi * elapsed_s / self._period_s
        T[self._axis_idx, 3] += self._amp_m * np.sin(phase)
        return clamp_workspace(self._filter.apply(T), limits=self._limits)

    @staticmethod
    def _choose_axis(T_center: np.ndarray, limits: dict) -> str:
        xyz = T_center[:3, 3]
        scores = {}
        for i, name in enumerate(("x", "y", "z")):
            lo, hi = limits[name]
            scores[name] = min(float(xyz[i] - lo), float(hi - xyz[i]))
        return max(scores, key=scores.get)


class MarvinArmATeleopDriver:
    """基于 TJ SDK 的 A 臂最小遥操驱动。"""

    def __init__(
        self,
        *,
        dummy: bool,
        robot_ip: str,
        config_path: Path,
        vel_ratio: int,
        acc_ratio: int,
        cart_k: list[float],
        cart_d: list[float],
        seed_joints_deg: list[float],
    ) -> None:
        _ensure_tj_sdk_path()
        from SDK_PYTHON.fx_kine import FX_InvKineSolvePara, Marvin_Kine  # type: ignore[import-not-found]  # noqa: PLC0415
        from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot  # type: ignore[import-not-found]  # noqa: PLC0415

        self._dummy = dummy
        self._robot_ip = robot_ip
        self._config_path = Path(config_path)
        self._vel_ratio = int(np.clip(vel_ratio, 1, 100))
        self._acc_ratio = int(np.clip(acc_ratio, 1, 100))
        self._cart_k = cart_k
        self._cart_d = cart_d
        self._last_q = list(seed_joints_deg)

        self._Marvin_Kine = Marvin_Kine
        self._FX_InvKineSolvePara = FX_InvKineSolvePara
        self._Marvin_Robot = Marvin_Robot
        self._DCSS = DCSS

        self._kine = None
        self._robot = None
        self._dcss = None
        self._last_T_m: Optional[np.ndarray] = None

    def connect(self) -> None:
        self._init_kine()
        if self._dummy:
            self._last_T_m = self._fk_m(self._last_q)
            logger.info("[dummy] 跳过控制柜连接，仅验证测试轨迹 + FK/IK")
            return

        self._robot = self._Marvin_Robot()
        self._dcss = self._DCSS()
        if self._robot.connect(self._robot_ip) != 1:
            raise RuntimeError("Marvin_Robot.connect 失败：检查 IP、网线或端口占用")

        self._robot.log_switch("0")
        self._robot.local_log_switch("0")
        self._clear_error_a()
        self._verify_frame_serial()
        self._enter_cart_impedance()

        q = self._read_joint_deg()
        if q is not None:
            self._last_q = q
        self._last_T_m = self._fk_m(self._last_q)
        logger.info(
            "A 臂已就绪：扭矩 + 笛卡尔阻抗，vel=%d%% acc=%d%%",
            self._vel_ratio,
            self._acc_ratio,
        )

    def disconnect(self) -> None:
        try:
            if not self._dummy and self._robot is not None:
                self._robot.clear_set()
                self._robot.set_state(arm="A", state=0)
                self._robot.send_cmd()
                time.sleep(0.1)
        finally:
            if not self._dummy and self._robot is not None:
                self._robot.release_robot()
                logger.info("A 臂已下使能并释放连接。")
            self._kine = None
            self._robot = None
            self._dcss = None

    def get_current_pose_m(self) -> np.ndarray:
        if not self._dummy:
            q = self._read_joint_deg()
            if q is not None:
                self._last_q = q
                self._last_T_m = self._fk_m(q)
        if self._last_T_m is None:
            self._last_T_m = self._fk_m(self._last_q)
        return self._last_T_m.copy()

    def servo_cartesian_m(self, T_target_m: np.ndarray) -> bool:
        q = self._ik_q(_m_T_to_sdk(T_target_m), self._last_q)
        if q is None:
            logger.warning("IK 失败，本帧保持。")
            return False

        self._last_q = q
        self._last_T_m = T_target_m.copy()
        if self._dummy:
            return True

        assert self._robot is not None
        self._robot.clear_set()
        self._robot.set_joint_cmd_pose(arm="A", joints=q)
        self._robot.send_cmd()
        return True

    def _init_kine(self) -> None:
        if not self._config_path.is_file():
            raise FileNotFoundError(f"MvKDCfg 不存在: {self._config_path}")
        kk = self._Marvin_Kine()
        kk.log_switch(0)
        ini = kk.load_config(arm_type=0, config_path=str(self._config_path))
        if not ini:
            raise RuntimeError(f"load_config 失败: {self._config_path}")
        ok = kk.initial_kine(
            robot_type=ini["TYPE"][0],
            dh=ini["DH"][0],
            pnva=ini["PNVA"][0],
            j67=ini["BD"][0],
        )
        if not ok:
            raise RuntimeError("initial_kine(A) 失败，请确认 MvKDCfg 与实物机型一致")
        self._kine = kk
        logger.info("Marvin_Kine A 臂初始化完成: %s", self._config_path.name)

    def _clear_error_a(self) -> None:
        assert self._robot is not None
        for i in range(2):
            self._robot.clear_set()
            self._robot.clear_error("A")
            self._robot.send_cmd()
            time.sleep(0.25)
            logger.info("A 臂清错脉冲 %d/2 已发送。", i + 1)

    def _verify_frame_serial(self) -> None:
        assert self._robot is not None and self._dcss is not None
        motion_tag = 0
        prev = None
        for _ in range(8):
            sub = self._robot.subscribe(self._dcss)
            fs = sub["outputs"][0]["frame_serial"]
            if fs != 0 and fs != prev:
                motion_tag += 1
                prev = fs
            time.sleep(0.02)
        if motion_tag == 0:
            raise RuntimeError("未检测到 A 臂 frame_serial 刷新，请查防火墙/网线/控制柜连接")

    def _enter_cart_impedance(self) -> None:
        assert self._robot is not None
        self._robot.clear_set()
        self._robot.set_cart_kd_params(arm="A", K=self._cart_k, D=self._cart_d, type=2)
        self._robot.send_cmd()
        time.sleep(0.25)

        self._robot.clear_set()
        self._robot.set_state(arm="A", state=3)
        self._robot.set_impedance_type(arm="A", type=2)
        self._robot.set_vel_acc(
            arm="A", velRatio=self._vel_ratio, AccRatio=self._acc_ratio
        )
        self._robot.send_cmd()
        time.sleep(0.35)

    def _read_joint_deg(self) -> Optional[list[float]]:
        if self._robot is None or self._dcss is None:
            return None
        sub = self._robot.subscribe(self._dcss)
        state = sub["states"][0]
        if state["cur_state"] == 100 or state["err_code"] != 0:
            logger.warning(
                "A 臂状态异常: cur_state=%s err_code=%s",
                state["cur_state"],
                state["err_code"],
            )
        return list(sub["outputs"][0]["fb_joint_pos"])

    def _fk_m(self, joints_deg: list[float]) -> np.ndarray:
        assert self._kine is not None
        fk = self._kine.fk(joints=list(joints_deg))
        if not fk:
            raise RuntimeError("FK 失败")
        return _sdk_T_to_m(np.array(fk, dtype=np.float64))

    def _ik_q(self, T_target_sdk: np.ndarray, ref_joints_deg: list[float]) -> Optional[list[float]]:
        assert self._kine is not None
        sp = self._FX_InvKineSolvePara()
        sp.set_input_ik_target_tcp(
            self._kine.mat4x4_to_mat1x16(_matrix4_to_rows(T_target_sdk))
        )
        sp.set_input_ik_ref_joint(ref_joints_deg)
        sp.set_input_ik_zsp_type(0)
        out = self._kine.ik(sp)
        if not out:
            return None
        if bool(out.m_Output_IsOutRange) or bool(out.m_Output_IsJntExd):
            logger.warning(
                "IK 输出越界: out_range=%s joint_exd=%s",
                bool(out.m_Output_IsOutRange),
                bool(out.m_Output_IsJntExd),
            )
            return None
        return out.m_Output_RetJoint.to_list()


def main() -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    ap = argparse.ArgumentParser(description="天机 Marvin A 臂纯测试（不接手）")
    ap.add_argument(
        "--mode",
        choices=("scripted", "pico"),
        default="scripted",
        help="scripted=无 PICO 自动小幅运动；pico=PICO 右手柄遥操",
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
        choices=("auto", "x", "-y", "z"),
        default="auto",
        help=(
            "scripted 模式的正弦运动方向；Marvin 基座系 X=前后/Y=上下/Z=左右，"
            "auto 选当前工作空间余量最大的轴"
        ),
    )
    ap.add_argument(
        "--amp-mm",
        type=float,
        default=5.0,
        help="scripted 模式的正弦幅值，单位 mm",
    )
    ap.add_argument(
        "--period",
        type=float,
        default=4.0,
        help="scripted 模式的正弦周期，单位秒",
    )
    ap.add_argument(
        "--max-step-mm",
        type=float,
        default=4.0,
        help="单拍最大平移目标变化，0 表示不限制",
    )
    ap.add_argument(
        "--max-rot-deg",
        type=float,
        default=3.0,
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
        "--controller",
        choices=("left", "right"),
        default="left",
        help=(
            "PICO 模式驱动 A 臂使用的手柄。Marvin 的 A 臂是左臂，默认用左手柄；"
            "右手柄请传 right。离合键自动切换：left=X 或左菜单，right=A 或右菜单。"
        ),
    )
    args = ap.parse_args()

    if args.hz <= 0:
        raise ValueError("--hz 必须 > 0")

    arm = MarvinArmATeleopDriver(
        dummy=args.dummy,
        robot_ip=args.ip,
        config_path=args.config if args.config is not None else _default_cfg(),
        vel_ratio=args.vel,
        acc_ratio=args.acc,
        cart_k=[2000, 2000, 2000, 40, 40, 40, 20],
        cart_d=[0.1, 0.1, 0.1, 0.3, 0.3, 0.3, 1.0],
        seed_joints_deg=args.seed_joints,
    )

    stop = {"flag": False}

    def _on_signal(signum, _frame):  # noqa: ANN001
        logger.info("收到信号 %s，准备退出...", signum)
        stop["flag"] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    arm.connect()
    last_cmd_T = arm.get_current_pose_m()
    limits = _build_workspace_limits(args.workspace_margin_m, last_cmd_T)

    xr: Optional[XrClient] = None
    streamer: Optional[PicoStreamer] = None
    if args.mode == "pico":
        xr = XrClient()
        xr.init()
        streamer = PicoStreamer(
            xr,
            workspace_limits=limits,
            side=args.controller,
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
            "开始 PICO -> A 臂遥操 @ %.1fHz；controller=%s，离合默认关闭，"
            "%s 切换离合，Ctrl+C 退出。",
            args.hz,
            args.controller,
            "X 或左菜单" if args.controller == "left" else "A 或右菜单",
        )
    else:
        logger.info(
            "开始无 PICO scripted A 臂测试 @ %.1fHz: axis=%s amp=%.1fmm period=%.1fs。Ctrl+C 退出。",
            args.hz,
            args.axis,
            args.amp_mm,
            args.period,
        )

    dt = 1.0 / args.hz
    next_tick = time.perf_counter()
    tick = 0
    ok_cnt = 0
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
                assert streamer is not None
                cmd = streamer.step(T_now)
                active = cmd.active
                T_target = cmd.T_target
                snap = streamer.last_snap
                if snap is None:
                    trigger = 0.0
                    grip = 0.0
                elif args.controller == "left":
                    trigger = snap.left_trigger
                    grip = snap.left_grip
                else:
                    trigger = snap.right_trigger
                    grip = snap.right_grip

            if active:
                T_limited = _limit_pose_step(
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
