"""PICO 模拟量 -> Revo2 灵巧手目标的映射工具。

Revo2 基础版 6 DoF 槽位（**以官方 ``revo2_timing_test_gui.py:FINGER_NAMES`` 为准**）::

    [Thumb Flex, Thumb Aux, Index, Middle, Ring, Pinky]

- **槽位 0 = Thumb Flex**：拇指**屈伸**（flexion）
- **槽位 1 = Thumb Aux**：拇指**对掌/内外收**（opposition/abduction）
- **槽位 2~5 = Index / Middle / Ring / Pinky**：四指屈曲，SDK 上限 1000

> 注：``revo2_ctrl.py`` 注释里笼统写作 ``[Thumb, ThumbAux, ...]``，没区分哪个是 Flex。
> 实机验证（按本工程 grip 映射到槽位 1 时看到的是内外收动作）确认
> Flex=0、Aux=1 才是真实语义。

本工程把这 6 个自由度拆成 6 个归一化量 ``[0, 1]``。

当前默认映射：

- ``gripper``：PICO ``trigger`` [0, 1] -> **五指**同步抓握，把 Revo2 当一维夹爪。
- ``two-channel``：PICO ``trigger`` 控四指，``grip`` 控拇指屈伸。
- 拇指内收/对掌（``thumb_opposition`` -> Thumb Aux，槽位 1）保持 ``main.py`` 注入的常量，
  默认 :data:`THUMB_OPPOSITION_DEFAULT` = 0.8，适合抓握类任务；可用命令行调到 1.0。

全程 0.1% 精度，位置范围 ``[0, 1000]``。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List


HAND_MODE_GRIPPER = "gripper"
HAND_MODE_TWO_CHANNEL = "two-channel"
HAND_MODES = (HAND_MODE_GRIPPER, HAND_MODE_TWO_CHANNEL)

# 拇指对掌默认值（归一化 [0, 1]；1.0 映射到 SDK_POS_THUMB_AUX_MAX）
THUMB_OPPOSITION_DEFAULT: float = 0.8


# SDK 端的 0.1% 精度量程
SDK_POS_MAX: int = 1000
SDK_POS_THUMB_FLEX_MAX: int = 700
SDK_POS_THUMB_AUX_MAX: int = 1000


def clamp(value: float, lo: float = 0.0, hi: float = 1.0) -> float:
    return max(lo, min(hi, float(value)))


@dataclass
class Revo2FingerTargets:
    """Revo2 六自由度归一化目标（0 完全伸直/外展 ~ 1 完全弯曲/对掌）。

    字段与 SDK 槽位一一对应：

    - ``thumb_bend``       -> Thumb Flex（槽位 0，拇指屈伸）
    - ``thumb_opposition`` -> Thumb Aux（槽位 1，拇指对掌/内外收）
    - ``index/middle/ring/pinky`` -> 槽位 2/3/4/5，四指屈曲
    """

    index: float
    middle: float
    ring: float
    pinky: float
    thumb_bend: float = 0.0
    thumb_opposition: float = THUMB_OPPOSITION_DEFAULT


@dataclass
class HandControlSample:
    """一帧手部控制/采集数据。

    这个对象是后续 recorder/RoboCOIN exporter 可以直接拿的接口：

    - ``side``：当前接的是左手还是右手。
    - ``mode``：手部控制模式，``gripper`` 或 ``two-channel``。
    - ``raw_trigger/raw_grip``：手柄原始连续输入。
    - ``grasp_scalar``：一维夹爪语义，0=全开，1=全抓。
    - ``targets``：Revo2 归一化 6D 目标。
    - ``hand_cmd_6d``：实际下发给 Revo2 SDK 的 6D 整数命令。
    """

    side: str
    mode: str
    raw_trigger: float
    raw_grip: float
    grasp_scalar: float
    targets: Revo2FingerTargets
    hand_cmd_6d: List[int]

    def as_record(self) -> dict:
        """返回便于 JSON/Parquet/LeRobot exporter 使用的扁平记录。"""
        return {
            "side": self.side,
            "mode": self.mode,
            "raw_trigger": self.raw_trigger,
            "raw_grip": self.raw_grip,
            "grasp_scalar": self.grasp_scalar,
            "hand_cmd_6d": list(self.hand_cmd_6d),
            "thumb_bend": self.targets.thumb_bend,
            "thumb_opposition": self.targets.thumb_opposition,
            "index": self.targets.index,
            "middle": self.targets.middle,
            "ring": self.targets.ring,
            "pinky": self.targets.pinky,
        }


def normalize_hand_mode(raw: str) -> str:
    mode = str(raw).strip().lower().replace("_", "-")
    if mode in {"grip", "grasp", "gripper", "one-channel", "one-d", "1d"}:
        return HAND_MODE_GRIPPER
    if mode in {"two-channel", "two", "2d", "dex", "legacy"}:
        return HAND_MODE_TWO_CHANNEL
    raise ValueError(f"未知 hand mode: {raw}，可选: {', '.join(HAND_MODES)}")


def compute_revo2_targets(
    trigger: float,
    grip: float,
    thumb_opposition: float = THUMB_OPPOSITION_DEFAULT,
    mode: str = HAND_MODE_TWO_CHANNEL,
) -> Revo2FingerTargets:
    """根据 PICO 模拟量计算 Revo2 目标。

    映射：

    - ``gripper``：``trigger`` -> 五指同步抓握，输出一维 ``grasp_scalar``。
    - ``two-channel``：``trigger`` -> 四指，``grip`` -> 拇指屈伸。
    - 拇指对掌 (``thumb_opposition``) 由调用方（通常是 ``main.py``）常量注入

    :param trigger: 手柄 trigger，[0, 1]，0=全开 1=全握
    :param grip:    手柄 grip，[0, 1]，two-channel 下用于拇指屈伸
    :param thumb_opposition: 拇指对掌量，[0, 1]
    :param mode: ``gripper`` 或 ``two-channel``
    """
    t = clamp(trigger, 0.0, 1.0)
    g = clamp(grip, 0.0, 1.0)
    opp = clamp(thumb_opposition, 0.0, 1.0)
    hand_mode = normalize_hand_mode(mode)
    thumb_bend = t if hand_mode == HAND_MODE_GRIPPER else g
    return Revo2FingerTargets(
        index=t,
        middle=t,
        ring=t,
        pinky=t,
        thumb_bend=thumb_bend,
        thumb_opposition=opp,
    )


def build_hand_control_sample(
    *,
    side: str,
    trigger: float,
    grip: float,
    thumb_opposition: float = THUMB_OPPOSITION_DEFAULT,
    mode: str = HAND_MODE_GRIPPER,
) -> HandControlSample:
    """从手柄输入构造一帧可下发、可记录的手部数据。"""
    if side not in {"left", "right"}:
        raise ValueError("side 只能是 'left' 或 'right'")
    hand_mode = normalize_hand_mode(mode)
    raw_trigger = clamp(trigger, 0.0, 1.0)
    raw_grip = clamp(grip, 0.0, 1.0)
    targets = compute_revo2_targets(
        trigger=raw_trigger,
        grip=raw_grip,
        thumb_opposition=thumb_opposition,
        mode=hand_mode,
    )
    return HandControlSample(
        side=side,
        mode=hand_mode,
        raw_trigger=raw_trigger,
        raw_grip=raw_grip,
        grasp_scalar=raw_trigger,
        targets=targets,
        hand_cmd_6d=to_sdk_positions(targets),
    )


def to_sdk_positions(targets: Revo2FingerTargets) -> List[int]:
    """把归一化目标转换为 Revo2 SDK 的 6 维位置数组。

    返回顺序：``[Thumb Flex, Thumb Aux, Index, Middle, Ring, Pinky]``，
    每项为 0.1% 精度的整数。Thumb Flex 和 Thumb Aux 分别按各自标定上限映射。
    """
    return [
        int(clamp(targets.thumb_bend) * SDK_POS_THUMB_FLEX_MAX),         # 槽位 0: Thumb Flex（屈伸）
        int(clamp(targets.thumb_opposition) * SDK_POS_THUMB_AUX_MAX),    # 槽位 1: Thumb Aux（对掌/内外收）
        int(clamp(targets.index) * SDK_POS_MAX),
        int(clamp(targets.middle) * SDK_POS_MAX),
        int(clamp(targets.ring) * SDK_POS_MAX),
        int(clamp(targets.pinky) * SDK_POS_MAX),
    ]


__all__ = [
    "HAND_MODE_GRIPPER",
    "HAND_MODE_TWO_CHANNEL",
    "HAND_MODES",
    "SDK_POS_MAX",
    "SDK_POS_THUMB_AUX_MAX",
    "SDK_POS_THUMB_FLEX_MAX",
    "THUMB_OPPOSITION_DEFAULT",
    "HandControlSample",
    "Revo2FingerTargets",
    "build_hand_control_sample",
    "clamp",
    "compute_revo2_targets",
    "normalize_hand_mode",
    "to_sdk_positions",
]
