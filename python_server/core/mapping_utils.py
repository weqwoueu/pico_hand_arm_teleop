"""PICO 模拟量 -> Revo2 灵巧手目标的映射工具。

Revo2 基础版 6 DoF 槽位（**以官方 ``revo2_timing_test_gui.py:FINGER_NAMES`` 为准**）::

    [Thumb Flex, Thumb Aux, Index, Middle, Ring, Pinky]

- **槽位 0 = Thumb Flex**：拇指**屈伸**（flexion），SDK 上限 500
- **槽位 1 = Thumb Aux**：拇指**对掌/内外收**（opposition/abduction），SDK 上限 500
- **槽位 2~5 = Index / Middle / Ring / Pinky**：四指屈曲，SDK 上限 1000

> 注：``revo2_ctrl.py`` 注释里笼统写作 ``[Thumb, ThumbAux, ...]``，没区分哪个是 Flex。
> 实机验证（按本工程 grip 映射到槽位 1 时看到的是内外收动作）确认
> Flex=0、Aux=1 才是真实语义。

本工程把这 6 个自由度拆成 6 个归一化量 ``[0, 1]``。

当前默认映射（main 里可改）：

- PICO ``right_trigger`` [0, 1] -> **四指**（食/中/无名/小）同步屈曲
- PICO ``right_grip``    [0, 1] -> **拇指屈伸**（``thumb_bend`` -> Thumb Flex，槽位 0）
- 拇指内收/对掌（``thumb_opposition`` -> Thumb Aux，槽位 1）保持 ``main.py`` 注入的常量，
  默认 :data:`THUMB_OPPOSITION_DEFAULT` = 0.80（≈ 400/500，适合抓握类任务）

全程 0.1% 精度，位置范围 ``[0, 1000]``；拇指两关节（Flex / Aux）SDK 内部量程上限约为 500。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List


# 拇指对掌默认值（归一化 [0, 1]；映射到 SDK 则为 ≈ 0.80 * 500 = 400）
THUMB_OPPOSITION_DEFAULT: float = 1.0

# SDK 端的 0.1% 精度量程
SDK_POS_MAX: int = 1000
SDK_POS_THUMB_MAX: int = 700  # Thumb / ThumbAux 两关节的上限


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


def compute_revo2_targets(
    trigger: float,
    grip: float,
    thumb_opposition: float = THUMB_OPPOSITION_DEFAULT,
) -> Revo2FingerTargets:
    """根据 PICO 模拟量计算 Revo2 目标。

    当前映射：

    - ``trigger`` -> 食/中/无名/小 四指同步屈曲
    - ``grip``    -> 拇指屈伸（``thumb_bend``）
    - 拇指对掌 (``thumb_opposition``) 由调用方（通常是 ``main.py``）常量注入

    :param trigger: 右手柄 trigger，[0, 1]，0=全开 1=全握
    :param grip:    右手柄 grip，[0, 1]，0=拇指伸 1=拇指全弯
    :param thumb_opposition: 拇指对掌量，[0, 1]，默认 0.80
    """
    t = clamp(trigger, 0.0, 1.0)
    g = clamp(grip, 0.0, 1.0)
    opp = clamp(thumb_opposition, 0.0, 1.0)
    return Revo2FingerTargets(
        index=t,
        middle=t,
        ring=t,
        pinky=t,
        thumb_bend=g,
        thumb_opposition=opp,
    )


def to_sdk_positions(targets: Revo2FingerTargets) -> List[int]:
    """把归一化目标转换为 Revo2 SDK 的 6 维位置数组。

    返回顺序：``[Thumb Flex, Thumb Aux, Index, Middle, Ring, Pinky]``，
    每项为 0.1% 精度的整数。Thumb Flex / Thumb Aux 实际量程上限 500，其余 1000。
    """
    return [
        int(clamp(targets.thumb_bend) * SDK_POS_THUMB_MAX),         # 槽位 0: Thumb Flex（屈伸）
        int(clamp(targets.thumb_opposition) * SDK_POS_THUMB_MAX),   # 槽位 1: Thumb Aux（对掌/内外收）
        int(clamp(targets.index) * SDK_POS_MAX),
        int(clamp(targets.middle) * SDK_POS_MAX),
        int(clamp(targets.ring) * SDK_POS_MAX),
        int(clamp(targets.pinky) * SDK_POS_MAX),
    ]
