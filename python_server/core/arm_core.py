"""Arm-only teleoperation and arm driver core.

This module intentionally keeps the reusable arm stack in one place:

- PICO controller -> arm target pose generation
- scripted target generation for hardware tests
- per-tick safety limiting
- Tianji Marvin A-arm SDK bridge with IK / NSP strategies

The Tianji-specific driver lives here for now, but the module boundary is named
``arm_core`` so future robot drivers can expose the same small surface without
changing tool scripts.
"""

from __future__ import annotations

import logging
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R

from .pico_streamer import (
    EMA_ALPHA_ROTATION,
    EMA_ALPHA_TRANSLATION,
    PoseEmaFilter,
    WORKSPACE_LIMITS,
    clamp_workspace,
    xr_pose_to_T,
)
from .xr_client import ControllerSnapshot, XrClient

logger = logging.getLogger(__name__)


# SDK demo 中常用的 A 臂演示构型；dummy 模式和无反馈兜底使用。
DEFAULT_SEED_JOINTS_DEG = [21.8, -41.0, -4.74, -63.67, 10.15, 14.72, 7.68]

# ---------------- 阻抗参数：现场主要改这里 ----------------
#
# 本模块现在有两个阻抗模式：
#   cart-impedance   坐标/笛卡尔阻抗
#   joint-impedance  关节阻抗
#
# 两个模式的上层链路完全一样：PICO/scripted -> TCP target -> IK -> set_joint_cmd_pose。
# 下面这些 K/D 只影响控制柜底层“跟目标时硬不硬、软不软、会不会抖”。
#
# 调参通用规律：
#   1. K 越大：越硬、越跟手，但更容易抖/冲/碰撞力大。
#   2. K 越小：越柔顺、安全余量更大，但会拖、末端/关节误差更明显。
#   3. D 越大：阻尼越强、抖动更容易压住，但太大可能发闷、滞后。
#   4. D 太小：响应更轻快，但容易晃、过冲。
#   5. 每次只改一组参数，先 scripted 小幅度，再 PICO；首测配合低 vel/acc、max-step。
#
# SDK 文档给的范围/建议：
#   关节阻抗:
#     K[0:7]  关节刚度，C++ 文档建议 <= 2，非负。
#     D[0:7]  关节阻尼，范围 0~1。
#     官方 demo 参考: K=[2,2,2,1.6,1,1,1], D=[0.3,0.3,0.3,0.2,0.2,0.2,0.2]
#   坐标阻抗:
#     K[0:3]  平移刚度 xyz，Python 文档写“不超过 3000”。
#     K[3:6]  旋转刚度 rxyz，Python 文档写“不超过 100”。
#     K[6]    零空间刚度，Python 文档写“不超过 20”。
#     D[0:7]  阻尼比例，范围 0~1。
#     官方 demo 参考: K=[2000,2000,2000,40,40,40,20], D=[0.1,0.1,0.1,0.3,0.3,0.3,1]
#
# 推荐起步：
#   - 新姿态/新工位/第一次试 joint-impedance，可以先降到：
#       JOINT_IMPEDANCE_K = [1.0, 1.0, 1.0, 0.8, 0.6, 0.6, 0.6]
#       JOINT_IMPEDANCE_D = [0.2, 0.2, 0.2, 0.15, 0.15, 0.15, 0.15]
#   - 要更跟手、更硬一点，再逐步靠近当前默认 joint 值。
#   - cart-impedance 当前默认保留现有实机手感，平移 K=5000 高于文档保守上限；
#     如果出现抖动/冲击/新场地首测，先降到官方 demo 的
#     [2000,2000,2000,40,40,40,20]。

CART_IMPEDANCE_K = [5000, 5000, 5000, 80, 80, 80, 20]
CART_IMPEDANCE_D = [0.35, 0.35, 0.35, 0.3, 0.3, 0.3, 1.0]

JOINT_IMPEDANCE_K = [2, 2, 2, 1.6, 1, 1, 1]
JOINT_IMPEDANCE_D = [0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2]


def repo_root() -> Path:
    return Path(__file__).resolve().parents[2]


def tianji_sdk_root() -> Path:
    return repo_root() / "third_party" / "TJ_FX_ROBOT_CONTRL_SDK"


def ensure_tianji_sdk_path() -> Path:
    root = tianji_sdk_root()
    if not root.is_dir():
        raise FileNotFoundError(f"未找到 TJ SDK 目录: {root}")
    root_s = str(root)
    if root_s not in sys.path:
        sys.path.insert(0, root_s)
    return root


def default_tianji_m6_config_path() -> Path:
    return tianji_sdk_root() / "DEMO_PYTHON" / "ccs_m6_31.MvKDCfg"


def normalize_ik_mode(raw: list[str]) -> tuple[str, str]:
    """Normalize CLI-like IK mode tokens.

    Supported forms:
    - ``normal``
    - ``nsp`` / ``nsp fixed``
    - ``nsp last``
    - ``nsp clutch``
    """
    parts = [str(x).strip().lower() for x in raw if str(x).strip()]
    if parts == ["normal"]:
        return "normal", "none"
    if not parts or parts[0] != "nsp":
        raise ValueError("只能是 normal、nsp、nsp fixed、nsp last 或 nsp clutch")
    if len(parts) == 1:
        return "nsp", "fixed"
    if len(parts) == 2 and parts[1] in {"fixed", "last", "clutch"}:
        return "nsp", parts[1]
    raise ValueError("nsp 后面只能接 fixed / last / clutch")


def _matrix4_to_rows(T: np.ndarray) -> list[list[float]]:
    if T.shape != (4, 4):
        raise ValueError(f"T shape must be (4, 4), got {T.shape}")
    return [[float(T[i, j]) for j in range(4)] for i in range(4)]


def _sdk_T_to_m(T_sdk: np.ndarray) -> np.ndarray:
    """SDK FK/IK 使用 mm 平移；项目内遥操核心使用 m 平移。"""
    T_m = T_sdk.copy()
    T_m[:3, 3] *= 0.001
    return T_m


def _m_T_to_sdk(T_m: np.ndarray) -> np.ndarray:
    T_sdk = T_m.copy()
    T_sdk[:3, 3] *= 1000.0
    return T_sdk


@dataclass
class ArmTeleopCommand:
    """一次 arm-only 遥操作指令。"""

    active: bool
    T_target: np.ndarray
    trigger: float = 0.0
    grip: float = 0.0
    vr_delta_m: np.ndarray | None = None


@dataclass
class ArmClutchState:
    active: bool = False
    last_button: bool = False
    T_vr_init: Optional[np.ndarray] = None
    T_ee_init: Optional[np.ndarray] = None


def pick_controller_inputs(
    snap: ControllerSnapshot, side: str
) -> tuple[np.ndarray, float, float, bool]:
    """抽取指定侧手柄的 pose / trigger / grip / 离合按键。"""
    if side == "right":
        return (
            snap.right_pose,
            float(snap.right_trigger),
            float(snap.right_grip),
            bool(snap.button_a or snap.button_menu),
        )
    if side == "left":
        return (
            snap.left_pose,
            float(snap.left_trigger),
            float(snap.left_grip),
            bool(snap.button_x or snap.left_button_menu),
        )
    raise ValueError("side 只能是 'left' 或 'right'")


class ArmTeleopController:
    """PICO 手柄到机械臂目标位姿的解耦状态机。"""

    def __init__(
        self,
        xr: XrClient,
        *,
        side: str = "left",
        workspace_limits: dict = WORKSPACE_LIMITS,
        ema_trans: float = EMA_ALPHA_TRANSLATION,
        ema_rot: float = EMA_ALPHA_ROTATION,
        translation_scale: float = 1.0,
        xyz_scale: np.ndarray | None = None,
        track_rotation: bool = False,
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError("side 只能是 'left' 或 'right'")
        self._xr = xr
        self._side = side
        self._workspace_limits = workspace_limits
        self._filter = PoseEmaFilter(alpha_trans=ema_trans, alpha_rot=ema_rot)
        self._clutch = ArmClutchState()
        self._translation_scale = float(translation_scale)
        self._xyz_scale = (
            np.ones(3, dtype=np.float64)
            if xyz_scale is None
            else np.asarray(xyz_scale, dtype=np.float64).reshape(3)
        )
        self._track_rotation = bool(track_rotation)
        self._last_snap: Optional[ControllerSnapshot] = None

    @property
    def last_snap(self) -> Optional[ControllerSnapshot]:
        return self._last_snap

    @property
    def side(self) -> str:
        return self._side

    @property
    def track_rotation(self) -> bool:
        return self._track_rotation

    def step(self, T_ee_now: np.ndarray) -> ArmTeleopCommand:
        """读取一帧 PICO 数据，输出本拍机械臂目标。"""
        try:
            return self._step_inner(T_ee_now)
        except Exception as exc:  # noqa: BLE001
            logger.warning("ArmTeleopController.step 本帧降级（%s）", exc)
            return ArmTeleopCommand(
                active=False,
                T_target=T_ee_now.copy(),
                vr_delta_m=np.zeros(3, dtype=np.float64),
            )

    def _step_inner(self, T_ee_now: np.ndarray) -> ArmTeleopCommand:
        snap = self._xr.snapshot()
        self._last_snap = snap

        ctrl_pose, trigger, grip, button_pressed = pick_controller_inputs(
            snap, self._side
        )
        T_vr_now = xr_pose_to_T(snap.headset_pose, ctrl_pose, side=self._side)

        self._update_clutch(button_pressed, T_vr_now, T_ee_now)

        vr_delta = np.zeros(3, dtype=np.float64)
        if (
            self._clutch.active
            and self._clutch.T_vr_init is not None
            and self._clutch.T_ee_init is not None
        ):
            T_cmd_raw = self._clutch.T_ee_init.copy()
            vr_delta = T_vr_now[:3, 3] - self._clutch.T_vr_init[:3, 3]
            cmd_delta = vr_delta * self._xyz_scale * self._translation_scale
            T_cmd_raw[:3, 3] = self._clutch.T_ee_init[:3, 3] + cmd_delta

            if self._track_rotation:
                R_delta = T_vr_now[:3, :3] @ self._clutch.T_vr_init[:3, :3].T
                T_cmd_raw[:3, :3] = R_delta @ self._clutch.T_ee_init[:3, :3]

            T_cmd = clamp_workspace(
                self._filter.apply(T_cmd_raw),
                limits=self._workspace_limits,
            )
        else:
            T_cmd = T_ee_now.copy()

        return ArmTeleopCommand(
            active=self._clutch.active,
            T_target=T_cmd,
            trigger=trigger,
            grip=grip,
            vr_delta_m=vr_delta,
        )

    def _update_clutch(
        self,
        button_pressed: bool,
        T_vr_now: np.ndarray,
        T_ee_now: np.ndarray,
    ) -> None:
        rising_edge = button_pressed and not self._clutch.last_button
        self._clutch.last_button = button_pressed

        if not rising_edge:
            return

        self._clutch.active = not self._clutch.active
        if self._clutch.active:
            self._clutch.T_vr_init = T_vr_now.copy()
            self._clutch.T_ee_init = T_ee_now.copy()
            self._filter.reset(T_ee_now)
            ee_xyz = T_ee_now[:3, 3]
            vr_xyz = T_vr_now[:3, 3]
            logger.info(
                "▶▶ arm-only 离合【激活】 ee_init=[%+.3f %+.3f %+.3f] vr_init=[%+.3f %+.3f %+.3f] rotation=%s",
                ee_xyz[0],
                ee_xyz[1],
                ee_xyz[2],
                vr_xyz[0],
                vr_xyz[1],
                vr_xyz[2],
                "follow" if self._track_rotation else "hold",
            )
        else:
            ee_xyz = T_ee_now[:3, 3]
            logger.info(
                "■■ arm-only 离合【释放】机械臂锁定于 ee=[%+.3f %+.3f %+.3f]",
                ee_xyz[0],
                ee_xyz[1],
                ee_xyz[2],
            )


def limit_pose_step(
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


def build_workspace_limits(margin_m: float, center_T: np.ndarray) -> dict:
    """根据当前末端位姿生成临时 bounding box。"""
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
    """无 PICO 测试输入源：在当前位姿附近生成小幅正弦目标。"""

    def __init__(
        self,
        *,
        center_T: np.ndarray,
        axis: str,
        amp_mm: float,
        period_s: float,
        limits: dict,
    ) -> None:
        if axis not in {"auto", "x", "y", "z", "-x", "-y", "-z"}:
            raise ValueError("--axis 只能是 auto/x/y/z/-x/-y/-z")
        if period_s <= 0.0:
            raise ValueError("--period 必须 > 0")

        self._limits = limits
        self._center_T = clamp_workspace(center_T, limits=limits)
        chosen_axis = self._choose_axis(self._center_T, limits) if axis == "auto" else axis
        self._sign = -1.0 if chosen_axis.startswith("-") else 1.0
        axis_name = chosen_axis[1:] if chosen_axis.startswith("-") else chosen_axis
        self._axis_idx = {"x": 0, "y": 1, "z": 2}[axis_name]
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
        T[self._axis_idx, 3] += self._sign * self._amp_m * np.sin(phase)
        return clamp_workspace(self._filter.apply(T), limits=self._limits)

    @staticmethod
    def _choose_axis(T_center: np.ndarray, limits: dict) -> str:
        xyz = T_center[:3, 3]
        scores = {}
        for i, name in enumerate(("x", "y", "z")):
            lo, hi = limits[name]
            scores[name] = min(float(xyz[i] - lo), float(hi - xyz[i]))
        return max(scores, key=scores.get)


class TianjiArmDriver:
    """基于 TJ SDK 的 A 臂最小遥操驱动。"""

    def __init__(
        self,
        *,
        dummy: bool,
        robot_ip: str,
        config_path: Path,
        vel_ratio: int,
        acc_ratio: int,
        arm_control_mode: str,
        cart_k: list[float],
        cart_d: list[float],
        joint_k: list[float],
        joint_d: list[float],
        seed_joints_deg: list[float],
        ik_debug: bool = False,
        ik_mode: str = "normal",
        ik_nsp_strategy: str = "fixed",
        ik_nsp_angle_deg: float = 0.0,
    ) -> None:
        ensure_tianji_sdk_path()
        from SDK_PYTHON.fx_kine import FX_InvKineSolvePara, Marvin_Kine  # type: ignore[import-not-found]  # noqa: PLC0415
        from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot  # type: ignore[import-not-found]  # noqa: PLC0415

        self._dummy = dummy
        self._robot_ip = robot_ip
        self._config_path = Path(config_path)
        self._vel_ratio = int(np.clip(vel_ratio, 1, 100))
        self._acc_ratio = int(np.clip(acc_ratio, 1, 100))
        self._arm_control_mode = arm_control_mode
        self._cart_k = cart_k
        self._cart_d = cart_d
        self._joint_k = joint_k
        self._joint_d = joint_d
        self._last_q = list(seed_joints_deg)
        self._ik_debug = bool(ik_debug)
        self._ik_mode = ik_mode
        self._ik_nsp_strategy = ik_nsp_strategy
        self._ik_nsp_angle_deg = float(ik_nsp_angle_deg)
        self._ik_zsp_para: Optional[list[float]] = None
        self._ik_zsp_seed: Optional[list[float]] = None
        self._ik_zsp_warned_bad_ref = False
        self._ik_reject_count = 0
        self._joint_lmt_n: Optional[np.ndarray] = None
        self._joint_lmt_p: Optional[np.ndarray] = None
        self._j67_lmt: Optional[np.ndarray] = None

        self._Marvin_Kine = Marvin_Kine
        self._FX_InvKineSolvePara = FX_InvKineSolvePara
        self._Marvin_Robot = Marvin_Robot
        self._DCSS = DCSS

        self._kine = None
        self._robot = None
        self._dcss = None
        self._last_T_m: Optional[np.ndarray] = None

    @property
    def ik_mode_label(self) -> str:
        if self._ik_mode != "nsp":
            return self._ik_mode
        return f"{self._ik_mode}:{self._ik_nsp_strategy}"

    @property
    def last_joints_deg(self) -> list[float]:
        return list(self._last_q)

    def connect(self) -> None:
        self._init_kine()
        if self._dummy:
            self._last_T_m = self._fk_m(self._last_q)
            self._init_ik_nsp(self._last_q)
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
        self._enter_control_mode()

        q = self._read_joint_deg()
        if q is not None:
            self._last_q = q
        self._last_T_m = self._fk_m(self._last_q)
        self._init_ik_nsp(self._last_q)
        logger.info(
            "A 臂已就绪：control=%s，vel=%d%% acc=%d%%",
            self._arm_control_mode,
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

    def capture_clutch_nsp_reference(self, joints_deg: list[float]) -> None:
        """离合激活瞬间刷新 NSP 参考平面。"""
        if self._ik_mode != "nsp" or self._ik_nsp_strategy != "clutch":
            return
        ok = self._set_ik_zsp_from_joints(joints_deg, reason="离合激活")
        if not ok:
            logger.warning("NSP clutch 参考平面刷新失败，本次继续使用上一参考平面。")

    def _init_kine(self) -> None:
        if not self._config_path.is_file():
            raise FileNotFoundError(f"MvKDCfg 不存在: {self._config_path}")
        kk = self._Marvin_Kine()
        kk.log_switch(0)
        ini = kk.load_config(arm_type=0, config_path=str(self._config_path))
        if not ini:
            raise RuntimeError(f"load_config 失败: {self._config_path}")
        pnva = np.asarray(ini["PNVA"][0], dtype=np.float64)
        self._joint_lmt_n = np.minimum(pnva[:, 0], pnva[:, 1])
        self._joint_lmt_p = np.maximum(pnva[:, 0], pnva[:, 1])
        self._j67_lmt = np.asarray(ini["BD"][0], dtype=np.float64)
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

    def _enter_control_mode(self) -> None:
        if self._arm_control_mode == "cart-impedance":
            self._enter_cart_impedance()
            return
        if self._arm_control_mode == "joint-impedance":
            self._enter_joint_impedance()
            return
        if self._arm_control_mode == "position":
            self._enter_position_mode()
            return
        raise ValueError(f"未知 arm_control_mode: {self._arm_control_mode}")

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

    def _enter_joint_impedance(self) -> None:
        assert self._robot is not None
        self._robot.clear_set()
        self._robot.set_state(arm="A", state=3)
        self._robot.set_impedance_type(arm="A", type=1)
        self._robot.set_vel_acc(
            arm="A", velRatio=self._vel_ratio, AccRatio=self._acc_ratio
        )
        self._robot.send_cmd()
        time.sleep(0.35)

        self._robot.clear_set()
        self._robot.set_joint_kd_params(arm="A", K=self._joint_k, D=self._joint_d)
        self._robot.send_cmd()
        time.sleep(0.25)

    def _enter_position_mode(self) -> None:
        assert self._robot is not None
        self._robot.clear_set()
        self._robot.set_vel_acc(
            arm="A", velRatio=self._vel_ratio, AccRatio=self._acc_ratio
        )
        self._robot.send_cmd()
        time.sleep(0.25)

        self._robot.clear_set()
        self._robot.set_state(arm="A", state=1)
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

    def _init_ik_nsp(self, seed_joints_deg: list[float]) -> None:
        if self._ik_mode != "nsp":
            return
        if self._ik_nsp_strategy not in {"fixed", "last", "clutch"}:
            raise ValueError(f"未知 NSP 策略: {self._ik_nsp_strategy}")
        if self._ik_nsp_strategy == "last":
            logger.info(
                "IK NSP 已启用：strategy=last，每帧用上一帧/当前反馈关节刷新参考臂角平面，angle=%+.1fdeg",
                self._ik_nsp_angle_deg,
            )
            return
        ok = self._set_ik_zsp_from_joints(
            seed_joints_deg,
            reason="启动初始" if self._ik_nsp_strategy == "fixed" else "启动兜底",
        )
        if not ok:
            logger.warning("NSP 初始参考平面不可用，已退回 normal IK")
            self._ik_mode = "normal"

    def _set_ik_zsp_from_joints(
        self,
        joints_deg: list[float],
        *,
        reason: str,
        log_success: bool = True,
    ) -> bool:
        assert self._kine is not None
        if abs(float(joints_deg[3])) < 0.5:
            if log_success:
                logger.warning(
                    "NSP %s参考姿态 J4 太接近 0 度，无法提取臂角平面",
                    reason,
                )
            return False
        fk_nsp = self._kine.fk_nsp(joints=list(joints_deg))
        if not fk_nsp:
            if log_success:
                logger.warning("fk_nsp 失败：无法用%s参考姿态提取肘平面", reason)
            return False
        _fk_mat, nsp_mat = fk_nsp
        nsp = np.asarray(nsp_mat, dtype=np.float64)
        self._ik_zsp_seed = list(joints_deg)
        self._ik_zsp_para = [
            float(nsp[0, 0]),
            float(nsp[1, 0]),
            float(nsp[2, 0]),
            0.0,
            0.0,
            0.0,
        ]
        if log_success:
            logger.info(
                "IK NSP 参考平面已设置：strategy=%s reason=%s seed=[%s] zsp_para=[%s] angle=%+.1fdeg",
                self._ik_nsp_strategy,
                reason,
                self._fmt_vec(np.asarray(joints_deg, dtype=np.float64)),
                self._fmt_vec(np.asarray(self._ik_zsp_para[:3], dtype=np.float64)),
                self._ik_nsp_angle_deg,
            )
        return True

    def _ik_q(self, T_target_sdk: np.ndarray, ref_joints_deg: list[float]) -> Optional[list[float]]:
        assert self._kine is not None
        if self._ik_mode == "nsp" and self._ik_nsp_strategy == "last":
            ok = self._set_ik_zsp_from_joints(
                ref_joints_deg,
                reason="上一帧",
                log_success=False,
            )
            if ok:
                self._ik_zsp_warned_bad_ref = False
            elif not self._ik_zsp_warned_bad_ref:
                logger.warning("NSP last 参考平面刷新失败，暂时沿用上一可用参考平面。")
                self._ik_zsp_warned_bad_ref = True

        sp = self._FX_InvKineSolvePara()
        sp.set_input_ik_target_tcp(
            self._kine.mat4x4_to_mat1x16(_matrix4_to_rows(T_target_sdk))
        )
        sp.set_input_ik_ref_joint(ref_joints_deg)
        if self._ik_mode == "nsp" and self._ik_zsp_para is not None:
            sp.set_input_ik_zsp_type(1)
            sp.set_input_ik_zsp_para(self._ik_zsp_para)
        else:
            sp.set_input_ik_zsp_type(0)
        out = self._kine.ik(sp)
        if out and self._ik_mode == "nsp" and self._ik_zsp_para is not None:
            sp.set_input_zsp_angle(self._ik_nsp_angle_deg)
            sp.set_dgr1(0.05)
            sp.set_dgr2(0.05)
            out = self._kine.ik_nsp(sp)
        if not out:
            return None
        if bool(out.m_Output_IsOutRange) or bool(out.m_Output_IsJntExd):
            self._log_ik_reject(out)
            return None
        return out.m_Output_RetJoint.to_list()

    def _log_ik_reject(self, out) -> None:  # noqa: ANN001
        self._ik_reject_count += 1
        ret = np.asarray(out.m_Output_RetJoint.to_list(), dtype=np.float64)
        run_p = np.asarray(out.m_Output_RunLmtP.to_list(), dtype=np.float64)
        run_n = np.asarray(out.m_Output_RunLmtN.to_list(), dtype=np.float64)
        tags = [bool(out.m_Output_JntExdTags[i]) for i in range(7)]
        violations = self._format_limit_violations(ret, run_n, run_p)

        if not violations and any(tags):
            violations = [
                f"J{i + 1} tag=true q={ret[i]:+.2f} range=({run_n[i]:+.2f},{run_p[i]:+.2f})"
                for i, tag in enumerate(tags)
                if tag
            ]

        detail = "; ".join(violations) if violations else "无逐轴越界细节"
        logger.warning(
            "IK 输出越界: out_range=%s joint_exd=%s exd_abs=%.3f；%s",
            bool(out.m_Output_IsOutRange),
            bool(out.m_Output_IsJntExd),
            float(out.m_Output_JntExdABS),
            detail,
        )

        if self._ik_debug or self._ik_reject_count <= 3 or self._ik_reject_count % 50 == 0:
            logger.warning(
                "IK ret_joint=[%s] run_n=[%s] run_p=[%s] tags=%s",
                self._fmt_vec(ret),
                self._fmt_vec(run_n),
                self._fmt_vec(run_p),
                tags,
            )
            self._log_all_ik_candidates(out)

    def _log_all_ik_candidates(self, out) -> None:  # noqa: ANN001
        if self._joint_lmt_n is None or self._joint_lmt_p is None:
            return
        n_result = max(0, min(int(out.m_OutPut_Result_Num), 8))
        if n_result == 0:
            logger.warning("IK all_joint 为空。")
            return
        all_q = np.asarray(out.m_OutPut_AllJoint.to_list(), dtype=np.float64).reshape(8, 8)
        for i in range(n_result):
            q = all_q[i, :7]
            lower, upper = self._candidate_run_limits(q)
            violations = self._format_limit_violations(q, lower, upper)
            verdict = "运行限位OK" if not violations else "; ".join(violations)
            logger.warning(
                "IK candidate[%d] q=[%s] -> %s",
                i,
                self._fmt_vec(q),
                verdict,
            )

    def _candidate_run_limits(self, q: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        assert self._joint_lmt_n is not None and self._joint_lmt_p is not None
        lower = self._joint_lmt_n.copy()
        upper = self._joint_lmt_p.copy()
        if self._j67_lmt is None:
            return lower, upper

        j6 = float(q[5])
        if j6 > 1.0:
            j6_eval = min(j6, float(upper[5]))
            pp = self._j67_lmt[0]
            pn = self._j67_lmt[3]
            upper[6] = min(float(upper[6]), self._poly2(pp, j6_eval))
            lower[6] = max(float(lower[6]), self._poly2(pn, j6_eval))
        elif j6 < -1.0:
            j6_eval = min(j6, float(upper[5]))
            np_lmt = self._j67_lmt[1]
            nn = self._j67_lmt[2]
            upper[6] = min(float(upper[6]), self._poly2(np_lmt, j6_eval))
            lower[6] = max(float(lower[6]), self._poly2(nn, j6_eval))
        return lower, upper

    @staticmethod
    def _poly2(coeff: np.ndarray, x: float) -> float:
        return float(coeff[0] * x * x + coeff[1] * x + coeff[2])

    @staticmethod
    def _format_limit_violations(
        q: np.ndarray,
        lower: np.ndarray,
        upper: np.ndarray,
        *,
        eps: float = 1e-6,
    ) -> list[str]:
        out: list[str] = []
        for i, (qi, lo, hi) in enumerate(zip(q, lower, upper, strict=True)):
            if qi < lo - eps:
                out.append(f"J{i + 1} {qi:+.2f} < {lo:+.2f} by {lo - qi:.2f}deg")
            elif qi > hi + eps:
                out.append(f"J{i + 1} {qi:+.2f} > {hi:+.2f} by {qi - hi:.2f}deg")
        return out

    @staticmethod
    def _fmt_vec(v: np.ndarray) -> str:
        return " ".join(f"{float(x):+.2f}" for x in v)


# Backward-compatible class name for older local experiments.
MarvinArmATeleopDriver = TianjiArmDriver
