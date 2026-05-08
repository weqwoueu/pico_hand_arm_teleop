"""天机 Marvin A 臂纯测试脚本（不连接灵巧手）。

机械臂侧只按 ``third_party/TJ_FX_ROBOT_CONTRL_SDK`` 官方 SDK 接口接入：

1. ``fx_robot.Marvin_Robot`` 负责连接、清错、订阅、切模式、下发关节命令；
2. ``fx_kine.Marvin_Kine`` 负责 FK/IK，PICO 笛卡尔目标先转 mm 再求 IK；
3. PICO 遥操层复用 ``core.arm_teleop``，先只做 arm-only 控制，和 Revo2 手解耦。

用法（在 ``python_server`` 目录）::

    # ① 不接控制柜：纯 IK / 轨迹链路自检
    ./run.sh -m tools.test_xr_to_arm --mode scripted --dummy --duration 3

    # ② 真机 + scripted：当前末端附近三轴自动正弦，校 X/Y/Z 物理方向
    export MARVIN_IP=192.168.71.190
    ./run.sh -m tools.test_xr_to_arm --mode scripted --axis auto --hz 20 --duration 6 \
        --amp-mm 5 --vel 5 --acc 5 --workspace-margin-m 0.05

    # ③ 真机 + PICO：A 臂是左臂，默认用左手柄 (X 或左菜单切离合)
    ./run.sh -m tools.test_xr_to_arm --mode pico --ik-mode nsp --track-rotation --controller left --hz 50 --vel 50 --acc 50         

    # ④ 对比底层控制模式：默认 cart-impedance，也可试 joint-impedance / position
    ./run.sh -m tools.test_xr_to_arm --mode pico --arm-control-mode joint-impedance \
        --controller left --hz 20 --vel 5 --acc 5 --workspace-margin-m 0.05

    # ⑤ 想用右手柄驱动 A 臂（不推荐）：
    ./run.sh -m tools.test_xr_to_arm --mode pico --controller right ...

操作：

- ``--mode scripted``：不需要 PICO，自动在当前末端位姿附近做小幅正弦；
- ``--mode pico``：用 ``--controller`` 选边：
  - ``left``  → 左手柄 pose / trigger / grip，**X 键** 或 **左菜单键** 切换离合；
  - ``right`` → 右手柄 pose / trigger / grip，**A 键** 或 **右菜单键** 切换离合；
- idle：只读 PICO 和臂状态，不下发；
- active：默认只跟随手柄平移增量，末端姿态保持离合激活瞬间不变；
- ``--arm-control-mode`` 只切换控制柜底层状态，上层 PICO -> TCP target -> IK -> joint cmd
  链路保持一致，方便对比 cart-impedance / joint-impedance / position；
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

# 兼容两种启动方式：
#   1) ./run.sh -m tools.test_xr_to_arm
#   2) python tools/test_xr_to_arm.py
_PKG_ROOT = Path(__file__).resolve().parent.parent
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from core.arm_teleop import (  # noqa: E402
    ArmTeleopController,
    ScriptedPoseSource,
    build_workspace_limits,
    limit_pose_step,
)
from core.pico_streamer import T_to_pos_rpy  # noqa: E402
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
    """SDK FK/IK 使用 mm 平移；项目内遥操核心使用 m 平移。"""
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


def _parse_vec3(raw: str) -> np.ndarray:
    vals = [float(x.strip()) for x in raw.split(",") if x.strip()]
    if len(vals) != 3:
        raise argparse.ArgumentTypeError("需要 3 个逗号分隔数值，例如 1,1,1")
    return np.asarray(vals, dtype=np.float64)


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
        arm_control_mode: str,
        cart_k: list[float],
        cart_d: list[float],
        joint_k: list[float],
        joint_d: list[float],
        seed_joints_deg: list[float],
        ik_debug: bool = False,
        ik_mode: str = "normal",
        ik_nsp_angle_deg: float = 0.0,
    ) -> None:
        _ensure_tj_sdk_path()
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
        self._ik_nsp_angle_deg = float(ik_nsp_angle_deg)
        self._ik_zsp_para: Optional[list[float]] = None
        self._ik_zsp_seed: Optional[list[float]] = None
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
        assert self._kine is not None
        if abs(float(seed_joints_deg[3])) < 0.5:
            logger.warning("NSP 参考姿态 J4 太接近 0 度，已退回 normal IK")
            self._ik_mode = "normal"
            return
        fk_nsp = self._kine.fk_nsp(joints=list(seed_joints_deg))
        if not fk_nsp:
            logger.warning("fk_nsp 失败：无法用当前初始姿态提取肘平面，已退回 normal IK")
            self._ik_mode = "normal"
            return
        _fk_mat, nsp_mat = fk_nsp
        nsp = np.asarray(nsp_mat, dtype=np.float64)
        self._ik_zsp_seed = list(seed_joints_deg)
        self._ik_zsp_para = [
            float(nsp[0, 0]),
            float(nsp[1, 0]),
            float(nsp[2, 0]),
            0.0,
            0.0,
            0.0,
        ]
        logger.info(
            "IK NSP 已启用：使用当前初始关节作为肘平面 seed=[%s] zsp_para=[%s] angle=%+.1fdeg",
            self._fmt_vec(np.asarray(seed_joints_deg, dtype=np.float64)),
            self._fmt_vec(np.asarray(self._ik_zsp_para[:3], dtype=np.float64)),
            self._ik_nsp_angle_deg,
        )

    def _ik_q(self, T_target_sdk: np.ndarray, ref_joints_deg: list[float]) -> Optional[list[float]]:
        assert self._kine is not None
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
        choices=("normal", "nsp"),
        default="normal",
        help="normal=普通 IK；nsp=启动时用当前关节 fk_nsp 提取肘平面，并用 ik_nsp 调整臂角",
    )
    ap.add_argument(
        "--ik-nsp-angle",
        type=float,
        default=0.0,
        help="NSP 臂角微调角度，单位 deg；左臂方向可先试 +15 / -15",
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
        arm_control_mode=args.arm_control_mode,
        cart_k=[5000, 5000, 5000, 80, 80, 80, 20],
        cart_d=[0.5, 0.35, 0.35, 0.3, 0.3, 0.3, 1.0],
        joint_k=[2, 2, 2, 1.6, 1, 1, 1],
        joint_d=[0.3, 0.3, 0.3, 0.2, 0.2, 0.2, 0.2],
        seed_joints_deg=args.seed_joints,
        ik_debug=args.ik_debug,
        ik_mode=args.ik_mode,
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
            args.ik_mode,
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
            args.ik_mode,
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
                assert arm_teleop is not None
                cmd = arm_teleop.step(T_now)
                active = cmd.active
                T_target = cmd.T_target
                trigger = cmd.trigger
                grip = cmd.grip

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
