"""PICO -> 机械臂的 arm-only 遥操核心。

本模块只负责生成机械臂目标位姿，不处理 Revo2 手指目标。它和
``pico_streamer.PicoStreamer`` 的区别是：默认把平移和姿态解耦。

- 平移：使用离合激活后 PICO 手柄在对应手/臂控制系下的 xyz 增量；
- 姿态：默认保持离合激活瞬间的末端姿态；
- 可选：``track_rotation=True`` 时再跟随手柄相对旋转。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
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
                R_delta = self._clutch.T_vr_init[:3, :3].T @ T_vr_now[:3, :3]
                T_cmd_raw[:3, :3] = self._clutch.T_ee_init[:3, :3] @ R_delta

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
