"""业务核心：位姿变换 / 离合 / EMA 滤波 / 工作空间限位。

该模块不直接操作硬件，只负责：

1. 将 PICO 采集到的原始 pose（头显 + 手柄）变换到机器人/手臂控制系下的齐次矩阵；
2. 根据离合按键实现"相对增量控制"：
   - 未激活时：机器人保持原位；
   - 激活瞬间（上升沿）：锁存当前手柄处理后位姿 ``T_vr_init`` 与机械臂末端真实位姿 ``T_ee_init``；
   - 激活期间：``T_cmd = T_ee_init @ inv(T_vr_init) @ T_vr_now``。
3. 对输出做一阶低通（EMA）滤波，消除人手抖动；
4. 对平移部分做立方体 bounding box 限位，防止撞桌子/自身基座。
5. 把 trigger / grip 转为 Revo2 手指目标弯曲量。
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp

from .mapping_utils import (
    compute_revo2_targets,
    Revo2FingerTargets,
    THUMB_OPPOSITION_DEFAULT,
)
from .xr_client import ControllerSnapshot, XrClient

logger = logging.getLogger(__name__)


# ---------- 常量与配置 ----------

# 工作空间限位（Marvin A 臂基座系，单位：米）
#
# 实测（fx_kine FK + 走点）确认 Marvin 基座系约定为 Y-up：
#   X = 前后（+X 向前，远离基座）
#   Y = 上下（+Y 向上）
#   Z = 左右横移（+Z 向操作者左侧；具体正方向以现场标定为准）
#
# 默认给一个「保守宽松」的 bounding box，能覆盖常见 FK 起点附近的小幅
# 遥操（包含 z≈0.8m 的伸出姿态），真实台架请按现场可达空间收紧。
WORKSPACE_LIMITS = {
    "x": (-1.00, 1.00),
    "y": (-1.00, 1.00),
    "z": (-1.00, 1.00),
}

# EMA 滤波系数（0~1，越大越灵敏；越小越平滑）
EMA_ALPHA_TRANSLATION = 0.25
EMA_ALPHA_ROTATION = 0.20

# PICO 轴系 -> Marvin 基座系 的基向量变换：
#
# PICO (OpenXR 约定)：右手系，X 向右，Y 向上，Z 向后（指向用户）。
# 机器人基座系（Marvin，已实测）：X 向前，Y 向上，Z 向操作者左侧。
#
# 约定：把 PICO 坐标系下的向量 v_p 映射到机器人坐标系下：
#   v_r = R_RP @ v_p
#
# 满足：
#   PICO 的 "前方"（-Z_p） -> 机器人的 "前方"（+X_r）
#   PICO 的 "上"  （+Y_p） -> 机器人的 "上"  （+Y_r）
#   PICO 的 "右"  （+X_p） -> 机器人的 "右"  （-Z_r）
#
# 即 R_RP 的列向量分别是 PICO 基向量在机器人系下的表达：
#   X_p -> -Z_r =>  ( 0, 0, -1)^T
#   Y_p -> +Y_r =>  ( 0, 1,  0)^T
#   Z_p -> -X_r =>  (-1, 0,  0)^T
R_ROBOT_FROM_PICO: np.ndarray = np.array(
    [
        [0.0, 0.0, -1.0],
        [0.0, 1.0, 0.0],
        [-1.0, 0.0, 0.0],
    ],
    dtype=np.float64,
)

# arm-only 遥操还需要在“基座系”之上区分左右手/左右臂的控制 frame。
#
# 当前约定：
#   - 右手 / 右臂沿用旧逻辑；
#   - 左手 / A 左臂的平移已实机验证，在旧逻辑基础上需要 Y 轴镜像，才符合
#     左臂坐标：+X 前、+Y 上、+Z 左。
#
# 旋转必须和同一个控制 frame 绑定，不能只在 tools 层对 xyz 乘符号。
# 对旋转使用 B @ R @ B.T 做基变换；即使 B 是镜像矩阵，结果仍是合法旋转矩阵。
LEFT_ARM_CONTROL_BASIS_FROM_LEGACY = np.diag([1.0, -1.0, 1.0])
RIGHT_ARM_CONTROL_BASIS_FROM_LEGACY = np.eye(3, dtype=np.float64)


@dataclass(frozen=True)
class PicoFrameProfile:
    """PICO pose 进入机器人控制前的左右手/左右臂坐标配置。"""

    side: str
    position_basis: np.ndarray
    rotation_basis: np.ndarray
    description: str


PICO_FRAME_PROFILES: dict[str, PicoFrameProfile] = {
    "right": PicoFrameProfile(
        side="right",
        position_basis=RIGHT_ARM_CONTROL_BASIS_FROM_LEGACY,
        rotation_basis=RIGHT_ARM_CONTROL_BASIS_FROM_LEGACY,
        description="legacy right-controller/right-arm frame",
    ),
    "left": PicoFrameProfile(
        side="left",
        position_basis=LEFT_ARM_CONTROL_BASIS_FROM_LEGACY,
        rotation_basis=LEFT_ARM_CONTROL_BASIS_FROM_LEGACY,
        description="left-controller/A-arm frame: +X front, +Y up, +Z left",
    ),
}


# ---------- 数学工具 ----------


def _quat_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """四元数(xyzw) -> 3x3 旋转矩阵。"""
    return R.from_quat([qx, qy, qz, qw]).as_matrix()


def _pose7_to_matrix(pose: np.ndarray) -> np.ndarray:
    """将 [x,y,z,qx,qy,qz,qw] 转为 4x4 齐次矩阵（仍在 PICO 坐标系）。"""
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _quat_to_matrix(pose[3], pose[4], pose[5], pose[6])
    T[:3, 3] = pose[:3]
    return T


def _frame_profile(side: str | PicoFrameProfile) -> PicoFrameProfile:
    if isinstance(side, PicoFrameProfile):
        return side
    try:
        return PICO_FRAME_PROFILES[side]
    except KeyError as exc:
        raise ValueError("side 只能是 'left' 或 'right'") from exc


def _apply_frame_profile(T: np.ndarray, profile: PicoFrameProfile) -> np.ndarray:
    """把 legacy robot-frame pose 映射到指定手/臂的控制 frame。"""
    T_out = T.copy()
    T_out[:3, 3] = profile.position_basis @ T[:3, 3]
    T_out[:3, :3] = profile.rotation_basis @ T[:3, :3] @ profile.rotation_basis.T
    return T_out


def _extract_yaw_rotation(R_mat: np.ndarray) -> np.ndarray:
    """提取绕机器人 +Y（重力反方向）轴的 Yaw 分量，返回 3x3 旋转矩阵。

    Marvin 基座系是 Y-up（X 前 / Y 上 / Z 横），所以 yaw（绕重力轴）
    等价于绕 +Y。把 headset 的"前向量"投影到机器人 X-Z 地平面，
    用 atan2 解出 yaw，再还原成 Ry(yaw)。
    """
    forward_r = R_mat[:, 0]
    # X 为前、Z 为横；地平面坐标取 (x, z)，注意右手系下 Ry 的方向约定
    yaw = float(np.arctan2(-forward_r[2], forward_r[0]))
    cy, sy = np.cos(yaw), np.sin(yaw)
    return np.array(
        [
            [cy, 0.0, sy],
            [0.0, 1.0, 0.0],
            [-sy, 0.0, cy],
        ],
        dtype=np.float64,
    )


def pose7_to_robot_pos_rpy(
    pose7: np.ndarray,
    *,
    side: str | PicoFrameProfile = "right",
) -> tuple[np.ndarray, np.ndarray]:
    """把 PICO 系下的 7D pose ``[x, y, z, qx, qy, qz, qw]`` 转到控制坐标系，
    返回 ``(xyz_m, rpy_deg)``。

    - ``xyz_m``: 形状 ``(3,)``，指定 ``side`` 控制系下的平移，单位米
    - ``rpy_deg``: 形状 ``(3,)``，指定 ``side`` 控制系下的 ``xyz`` 欧拉角，单位度

    注意这里返回的是**绝对位姿**（纯基变换），没做"减头显原点 / 去偏航"那一套，
    因此适合做诊断/调试打印；业务侧的相对量请用 :func:`xr_pose_to_T`。
    """
    T_p = _pose7_to_matrix(pose7)
    T_r = np.eye(4, dtype=np.float64)
    T_r[:3, :3] = R_ROBOT_FROM_PICO @ T_p[:3, :3] @ R_ROBOT_FROM_PICO.T
    T_r[:3, 3] = R_ROBOT_FROM_PICO @ T_p[:3, 3]
    T_ctrl = _apply_frame_profile(T_r, _frame_profile(side))
    rpy = R.from_matrix(T_ctrl[:3, :3]).as_euler("xyz", degrees=True)
    return T_ctrl[:3, 3], rpy


def T_to_pos_rpy(T: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """把 4x4 齐次矩阵拆成 ``(xyz_m, rpy_deg)``，方便上层打印/记录。"""
    xyz = T[:3, 3].copy()
    rpy = R.from_matrix(T[:3, :3]).as_euler("xyz", degrees=True)
    return xyz, rpy


def xr_pose_to_T(
    headset_pose7: np.ndarray,
    ctrl_pose7: np.ndarray,
    *,
    side: str | PicoFrameProfile = "right",
) -> np.ndarray:
    """把头显 + 手柄的 7D pose 变换为机器人/手臂控制系下的 4x4 齐次矩阵。

    步骤：
    1. 将两个 pose 从 "PICO 轴系 (Y-up, Z 向后)" 转换到 "机器人基座系
       (X 前 / Y 上 / Z 横，Y-up)"；
    2. 计算手柄相对于 headset 的相对位姿：
           p_rel = p_ctrl - p_head
           R_rel = R_ctrl  （此处仅以 headset 位置作为参考原点，朝向处理放在下一步）
    3. 以 headset 的 Yaw 做"去偏航"（原地转身不动手也不该让末端乱飞）：
           p_out = R_inv_yaw @ p_rel
           R_out = R_inv_yaw @ R_rel
    4. 将 (p_out, R_out) 拼成 4x4 齐次矩阵；
    5. 根据 ``side`` 应用左右手/左右臂控制 frame 修正。
    """
    profile = _frame_profile(side)

    # ---- 1. 把 pose 从 PICO 系 变到 机器人系 ----
    # 设 T_PR 为 "机器人系下的 4x4"，由 R_RP 把 PICO 基向量旋转过来：
    #   R_r = R_RP @ R_p @ R_RP^T   （基变换）
    #   p_r = R_RP @ p_p
    R_RP = R_ROBOT_FROM_PICO

    T_head_p = _pose7_to_matrix(headset_pose7)
    T_ctrl_p = _pose7_to_matrix(ctrl_pose7)

    def transform_basis(T_p: np.ndarray) -> np.ndarray:
        T_r = np.eye(4, dtype=np.float64)
        T_r[:3, :3] = R_RP @ T_p[:3, :3] @ R_RP.T
        T_r[:3, 3] = R_RP @ T_p[:3, 3]
        return T_r

    T_head_r = transform_basis(T_head_p)
    T_ctrl_r = transform_basis(T_ctrl_p)

    # ---- 2. 以 headset 位置为参考原点，计算相对位姿 ----
    p_head = T_head_r[:3, 3]
    p_ctrl = T_ctrl_r[:3, 3]
    R_ctrl = T_ctrl_r[:3, :3]

    p_rel = p_ctrl - p_head
    R_rel = R_ctrl  # 仅以 headset 的位置作原点；朝向在第 3 步单独处理

    # ---- 3. 去偏航：抽取 headset 的 yaw，做其逆 ----
    R_head = T_head_r[:3, :3]
    R_yaw = _extract_yaw_rotation(R_head)
    R_inv_yaw = R_yaw.T  # 旋转矩阵的逆等于转置

    p_out = R_inv_yaw @ p_rel
    R_out = R_inv_yaw @ R_rel

    # ---- 4. 拼装齐次矩阵 ----
    T_out = np.eye(4, dtype=np.float64)
    T_out[:3, :3] = R_out
    T_out[:3, 3] = p_out
    return _apply_frame_profile(T_out, profile)


# ---------- EMA 滤波器 ----------


class PoseEmaFilter:
    """对 4x4 齐次矩阵做分解后的 EMA 滤波：

    - 平移部分：逐分量线性低通 ``x = a * x_new + (1-a) * x``；
    - 旋转部分：用 Slerp 做四元数球面线性插值（等价于"旋转空间的一阶低通"）。
    """

    def __init__(
        self,
        alpha_trans: float = EMA_ALPHA_TRANSLATION,
        alpha_rot: float = EMA_ALPHA_ROTATION,
    ) -> None:
        self.alpha_t = float(np.clip(alpha_trans, 0.0, 1.0))
        self.alpha_r = float(np.clip(alpha_rot, 0.0, 1.0))
        self._p: Optional[np.ndarray] = None
        self._q: Optional[np.ndarray] = None  # [x,y,z,w]

    def reset(self, T_init: Optional[np.ndarray] = None) -> None:
        if T_init is None:
            self._p = None
            self._q = None
        else:
            self._p = T_init[:3, 3].copy()
            self._q = R.from_matrix(T_init[:3, :3]).as_quat()

    def apply(self, T_new: np.ndarray) -> np.ndarray:
        p_new = T_new[:3, 3]
        q_new = R.from_matrix(T_new[:3, :3]).as_quat()

        if self._p is None or self._q is None:
            self._p = p_new.copy()
            self._q = q_new.copy()
            return T_new.copy()

        # 平移：一阶低通
        self._p = self.alpha_t * p_new + (1.0 - self.alpha_t) * self._p

        # 旋转：Slerp(q_prev, q_new, alpha_r)
        # Slerp 需要两个关键帧时间戳，这里用 [0, 1] 并插值在 alpha_r
        key_rots = R.from_quat(np.stack([self._q, q_new], axis=0))
        slerp = Slerp([0.0, 1.0], key_rots)
        q_interp = slerp([self.alpha_r]).as_quat()[0]
        self._q = q_interp

        T_out = np.eye(4, dtype=np.float64)
        T_out[:3, :3] = R.from_quat(self._q).as_matrix()
        T_out[:3, 3] = self._p
        return T_out


def clamp_workspace(T: np.ndarray, limits: dict = WORKSPACE_LIMITS) -> np.ndarray:
    """对齐次矩阵的平移做 bounding box 钳位。"""
    T_out = T.copy()
    x, y, z = T_out[:3, 3]
    T_out[0, 3] = float(np.clip(x, limits["x"][0], limits["x"][1]))
    T_out[1, 3] = float(np.clip(y, limits["y"][0], limits["y"][1]))
    T_out[2, 3] = float(np.clip(z, limits["z"][0], limits["z"][1]))
    return T_out


# ---------- 离合 + 主业务类 ----------


@dataclass
class TeleopCommand:
    """一次遥操作下发指令。"""

    active: bool
    T_target: np.ndarray
    fingers: Revo2FingerTargets


@dataclass
class ClutchState:
    active: bool = False
    last_button: bool = False
    T_vr_init: Optional[np.ndarray] = None
    T_ee_init: Optional[np.ndarray] = None


class PicoStreamer:
    """业务核心层：拉取手柄数据，产出机械臂 + 灵巧手指令。

    通过 ``side`` 选择驱动臂的手柄：

    - ``side="right"``：右手柄 pose / trigger / grip / A 或右菜单键。
    - ``side="left"`` ：左手柄 pose / trigger / grip / X  或左菜单键。

    Marvin 的 A 臂是左臂、B 臂是右臂，约定上更顺手是同侧映射。
    """

    def __init__(
        self,
        xr: XrClient,
        ema_trans: float = EMA_ALPHA_TRANSLATION,
        ema_rot: float = EMA_ALPHA_ROTATION,
        workspace_limits: dict = WORKSPACE_LIMITS,
        thumb_opposition: float = THUMB_OPPOSITION_DEFAULT,
        side: str = "right",
    ) -> None:
        if side not in {"left", "right"}:
            raise ValueError("side 只能是 'left' 或 'right'")
        self._xr = xr
        self._filter = PoseEmaFilter(alpha_trans=ema_trans, alpha_rot=ema_rot)
        self._clutch = ClutchState()
        self._workspace_limits = workspace_limits
        self._thumb_opposition = float(thumb_opposition)
        self._side = side
        # 供上层诊断 / 打印使用，避免 main 再次 snapshot
        self._last_snap: Optional[ControllerSnapshot] = None

    @property
    def last_snap(self) -> Optional[ControllerSnapshot]:
        """最近一次 ``step()`` 拉到的 PICO 采样快照（主循环调试用）。"""
        return self._last_snap

    @property
    def side(self) -> str:
        return self._side

    def _pick_inputs(
        self, snap: ControllerSnapshot
    ) -> tuple[np.ndarray, float, float, bool]:
        """根据 ``self._side`` 抽取该侧手柄的：pose / trigger / grip / 离合按键。

        离合按键：右侧用 A 或 右菜单；左侧用 X 或 左菜单（任一上升沿即触发）。
        """
        if self._side == "right":
            return (
                snap.right_pose,
                snap.right_trigger,
                snap.right_grip,
                bool(snap.button_a or snap.button_menu),
            )
        return (
            snap.left_pose,
            snap.left_trigger,
            snap.left_grip,
            bool(snap.button_x or snap.left_button_menu),
        )

    # --- 离合按键状态机 ---
    def _update_clutch(
        self,
        button_pressed: bool,
        T_vr_now: np.ndarray,
        T_ee_now: np.ndarray,
    ) -> None:
        rising_edge = button_pressed and not self._clutch.last_button
        self._clutch.last_button = button_pressed

        if rising_edge:
            self._clutch.active = not self._clutch.active
            if self._clutch.active:
                self._clutch.T_vr_init = T_vr_now.copy()
                self._clutch.T_ee_init = T_ee_now.copy()
                self._filter.reset(T_ee_now)
                ee_xyz = T_ee_now[:3, 3]
                vr_xyz = T_vr_now[:3, 3]
                logger.info(
                    "▶▶ 离合【激活】  ee_init=[%+.3f %+.3f %+.3f]  vr_init=[%+.3f %+.3f %+.3f]",
                    ee_xyz[0], ee_xyz[1], ee_xyz[2],
                    vr_xyz[0], vr_xyz[1], vr_xyz[2],
                )
            else:
                ee_xyz = T_ee_now[:3, 3]
                logger.info(
                    "■■ 离合【释放】  机械臂锁定于 ee=[%+.3f %+.3f %+.3f]",
                    ee_xyz[0], ee_xyz[1], ee_xyz[2],
                )

    # --- 主 step ---
    def step(self, T_ee_now: np.ndarray) -> TeleopCommand:
        """每次主循环调用一次，返回本拍要下发给硬件的指令。

        为了保护 100Hz 主循环不被偶发数据异常打断，本函数用 try/except
        兜底：任何计算异常（如 SDK 尚未就绪、四元数零范、矩阵奇异等）
        都退化为"离合未激活 + 保持当前位姿 + 零手指"，只打 warn 不抛。
        """
        try:
            return self._step_inner(T_ee_now)
        except Exception as exc:  # noqa: BLE001
            logger.warning("PicoStreamer.step 本帧降级（%s）", exc)
            return TeleopCommand(
                active=False,
                T_target=T_ee_now.copy(),
                fingers=compute_revo2_targets(
                    0.0, 0.0, thumb_opposition=self._thumb_opposition
                ),
            )

    def _step_inner(self, T_ee_now: np.ndarray) -> TeleopCommand:
        snap = self._xr.snapshot()
        self._last_snap = snap

        ctrl_pose, trigger, grip, button_pressed = self._pick_inputs(snap)

        # 1. VR 侧处理位姿（去偏航 + 基变换 + 左/右手控制 frame）
        T_vr_now = xr_pose_to_T(snap.headset_pose, ctrl_pose, side=self._side)

        # 2. 离合状态机
        self._update_clutch(button_pressed, T_vr_now, T_ee_now)

        # 3. 根据离合状态计算目标位姿
        if (
            self._clutch.active
            and self._clutch.T_vr_init is not None
            and self._clutch.T_ee_init is not None
        ):
            # 相对增量：T_cmd = T_ee_init @ inv(T_vr_init) @ T_vr_now
            T_cmd_raw = (
                self._clutch.T_ee_init
                @ np.linalg.inv(self._clutch.T_vr_init)
                @ T_vr_now
            )
            T_cmd_filtered = self._filter.apply(T_cmd_raw)
            T_cmd = clamp_workspace(T_cmd_filtered, self._workspace_limits)
        else:
            # 非激活：沿用机械臂当前位姿（等价于不动）
            T_cmd = T_ee_now.copy()

        # 4. 手指目标
        #    - trigger -> 食/中/无名/小 四指屈曲
        #    - grip    -> 拇指屈伸
        #    - 拇指对掌由上层（main）注入常量
        fingers = compute_revo2_targets(
            trigger,
            grip,
            thumb_opposition=self._thumb_opposition,
        )

        return TeleopCommand(
            active=self._clutch.active,
            T_target=T_cmd,
            fingers=fingers,
        )
