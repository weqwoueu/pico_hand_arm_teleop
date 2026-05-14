"""PICO 遥操作主入口：100Hz 主循环。

三层解耦：
    XrClient  ->  PicoStreamer  ->  TianjiRevoHardwareNode

运行时 TUI：Rich ``Live`` 在终端底部原地刷新状态表；``logging`` 走
``RichHandler`` 在面板上方自然滚动（不会把面板冲掉）。
"""

from __future__ import annotations

import logging
import os
import signal
import sys
import time
from pathlib import Path
from typing import Optional

from rich.console import Console
from rich.live import Live
from rich.logging import RichHandler
from rich.panel import Panel
from rich.table import Table
from rich.text import Text

try:
    from core.hardware_node import TianjiRevoHardwareNode
    from core.mapping_utils import to_sdk_positions
    from core.pico_streamer import PicoStreamer, T_to_pos_rpy, pose7_to_robot_pos_rpy
    from core.xr_client import ControllerSnapshot, XrClient
except ImportError:
    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from core.hardware_node import TianjiRevoHardwareNode
    from core.mapping_utils import to_sdk_positions
    from core.pico_streamer import PicoStreamer, T_to_pos_rpy, pose7_to_robot_pos_rpy
    from core.xr_client import ControllerSnapshot, XrClient


LOOP_HZ = 100
LOOP_DT = 1.0 / LOOP_HZ

# 状态面板刷新频率（越大越费 CPU，太低看起来卡。10~20 刚好）
TUI_REFRESH_HZ = 15

# 主循环每 N 拍触发一次 Live 重绘；100Hz / 5 ≈ 20Hz 画面
TUI_UPDATE_EVERY_N_TICKS = 5

# ---- 用户可调常量 ----
# 拇指 Thumb Aux（对掌 / 内外收）固定量，归一化 [0, 1]：
#   0.0 = 拇指完全外展（侧立，远离掌心）
#   1.0 = 拇指完全内收 / 对掌（横过掌心）
# 不随手柄变化，仅由此常量决定；想让拇指伸平就调小、想抓握就调大。
# 抓取类任务通常用 0.7~0.9。
THUMB_OPPOSITION: float = 1.00

# 与 ``PicoStreamer(..., side=...)``、``xr_pose_to_T(..., side=...)`` 对齐。
# 天机 A 臂在左侧时常用左手柄：``export TELEOP_CONTROLLER_SIDE=left``。
TELEOP_CONTROLLER_SIDE: str = os.environ.get("TELEOP_CONTROLLER_SIDE", "right").strip().lower()
if TELEOP_CONTROLLER_SIDE not in {"left", "right"}:
    raise ValueError("环境变量 TELEOP_CONTROLLER_SIDE 只能为 left 或 right")

# 全局 console，供 logging handler 和 Live 共用
_CONSOLE = Console()


def setup_logging() -> None:
    """把 logging 绑到 Rich 上，这样打 log 不会撕裂 Live 面板。"""
    logging.basicConfig(
        level=logging.INFO,
        format="%(name)s: %(message)s",
        datefmt="%H:%M:%S",
        handlers=[
            RichHandler(
                console=_CONSOLE,
                show_time=True,
                show_path=False,
                markup=False,
                rich_tracebacks=True,
            )
        ],
    )


# ---------- 状态面板 ----------


def _bool_tag(v: bool, label_true: str = "●", label_false: str = "·") -> Text:
    return Text(label_true, style="bold green") if v else Text(label_false, style="dim")


def _bar(value: float, width: int = 16) -> Text:
    """把 [0,1] 画成一条 ▇ 条。"""
    value = max(0.0, min(1.0, float(value)))
    filled = int(round(value * width))
    t = Text()
    t.append("▇" * filled, style="cyan")
    t.append("░" * (width - filled), style="dim")
    return t


def build_status_panel(
    tick: int,
    snap: Optional[ControllerSnapshot],
    active: bool,
    sdk_positions: Optional[list] = None,
    T_target: Optional["np.ndarray"] = None,
    *,
    teleop_side: str = "right",
) -> Panel:
    table = Table.grid(padding=(0, 2), expand=False)
    table.add_column(justify="right", style="cyan", no_wrap=True)
    table.add_column(justify="left")

    table.add_row("tick", f"{tick}  ({LOOP_HZ} Hz)")
    table.add_row("teleop side", teleop_side)

    if snap is None:
        table.add_row("status", Text("等待 PICO 数据 …", style="yellow"))
        return Panel(table, title="PICO Teleop", border_style="cyan")

    # 离合（active 时整条绿色高亮；idle 时暗色，与 arm 行颜色一致，方便一眼看到效果）
    clutch_txt = (
        Text("▶ ACTIVE — 手柄增量跟随中", style="bold black on green")
        if active
        else Text("■ idle   — 机械臂锁定", style="dim")
    )
    table.add_row("clutch", clutch_txt)

    # 按键（与 teleop_side 一致：右手 A/Menu，左手 X/左 Menu）
    btn_row = Text()
    if teleop_side == "right":
        btn_row.append_text(_bool_tag(snap.button_a))
        btn_row.append(" A   ")
        btn_row.append_text(_bool_tag(snap.button_menu))
        btn_row.append(" Menu")
    else:
        btn_row.append_text(_bool_tag(snap.button_x))
        btn_row.append(" X   ")
        btn_row.append_text(_bool_tag(snap.left_button_menu))
        btn_row.append(" L.Menu")
    table.add_row("buttons", btn_row)

    trig = snap.right_trigger if teleop_side == "right" else snap.left_trigger
    grip = snap.right_grip if teleop_side == "right" else snap.left_grip
    trig_txt = Text()
    trig_txt.append_text(_bar(trig))
    trig_txt.append(f"  {trig:.2f}")
    grip_txt = Text()
    grip_txt.append_text(_bar(grip))
    grip_txt.append(f"  {grip:.2f}")
    table.add_row("trigger", trig_txt)
    table.add_row("grip", grip_txt)

    # 头显 / 手柄位姿（机器人基座系；手柄须与 PicoStreamer.side 一致）
    ctrl_pose = snap.right_pose if teleop_side == "right" else snap.left_pose
    h_xyz, h_rpy = pose7_to_robot_pos_rpy(snap.headset_pose)
    c_xyz, c_rpy = pose7_to_robot_pos_rpy(ctrl_pose, side=teleop_side)
    table.add_row(
        "head xyz (m)",
        f"[ {h_xyz[0]:+6.3f}  {h_xyz[1]:+6.3f}  {h_xyz[2]:+6.3f} ]",
    )
    table.add_row(
        "head rpy (°)",
        f"[ {h_rpy[0]:+6.1f}  {h_rpy[1]:+6.1f}  {h_rpy[2]:+6.1f} ]",
    )
    table.add_row(
        "ctrl xyz (m)",
        f"[ {c_xyz[0]:+6.3f}  {c_xyz[1]:+6.3f}  {c_xyz[2]:+6.3f} ]",
    )
    table.add_row(
        "ctrl rpy (°)",
        f"[ {c_rpy[0]:+6.1f}  {c_rpy[1]:+6.1f}  {c_rpy[2]:+6.1f} ]",
    )

    # 机械臂目标（离合激活 -> 手柄增量已叠加；离合 idle -> 锁定值）
    # active 时绿色高亮：这一行跟着你的手动，说明离合生效
    # idle   时灰色：这一行冻结，无论你怎么挥手都不变
    if T_target is not None:
        a_xyz, a_rpy = T_to_pos_rpy(T_target)
        arm_style = "bold green" if active else "dim"
        table.add_row(
            Text("arm xyz (m)", style=arm_style),
            Text(
                f"[ {a_xyz[0]:+6.3f}  {a_xyz[1]:+6.3f}  {a_xyz[2]:+6.3f} ]",
                style=arm_style,
            ),
        )
        table.add_row(
            Text("arm rpy (°)", style=arm_style),
            Text(
                f"[ {a_rpy[0]:+6.1f}  {a_rpy[1]:+6.1f}  {a_rpy[2]:+6.1f} ]",
                style=arm_style,
            ),
        )

    # 六指 SDK 值
    if sdk_positions is not None and len(sdk_positions) == 6:
        tf, ta, i, m, r, p = sdk_positions
        table.add_row(
            "fingers SDK",
            f"TFlex={tf:3d}  TAux={ta:3d}  I={i:4d}  M={m:4d}  R={r:4d}  P={p:4d}",
        )

    return Panel(
        table,
        title="[bold]PICO → Revo2 Teleop[/bold]",
        subtitle="[dim]Ctrl+C 退出[/dim]",
        border_style="cyan",
    )


# ---------- 主循环 ----------


def main() -> None:
    setup_logging()
    log = logging.getLogger("teleop")

    xr = XrClient()
    hw = TianjiRevoHardwareNode()
    streamer = PicoStreamer(
        xr, thumb_opposition=THUMB_OPPOSITION, side=TELEOP_CONTROLLER_SIDE
    )

    xr.init()
    hw.connect()
    log.info(
        "遥操作主循环启动 (%.0f Hz)，手柄侧=%s（可用 TELEOP_CONTROLLER_SIDE=left|right 覆盖）。按 Ctrl+C 退出。",
        LOOP_HZ,
        TELEOP_CONTROLLER_SIDE,
    )

    stop_flag = {"stop": False}

    def _on_signal(signum, _frame):  # noqa: ANN001
        log.info("收到信号 %s，准备退出...", signum)
        stop_flag["stop"] = True

    signal.signal(signal.SIGINT, _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    next_tick = time.perf_counter()
    tick = 0

    try:
        with Live(
            build_status_panel(
                tick, None, False, None, teleop_side=TELEOP_CONTROLLER_SIDE
            ),
            console=_CONSOLE,
            refresh_per_second=TUI_REFRESH_HZ,
            transient=False,
        ) as live:
            while not stop_flag["stop"]:
                T_ee_now = hw.get_current_ee_pose()
                cmd = streamer.step(T_ee_now)

                if cmd.active:
                    hw.servo_cartesian(cmd.T_target)

                hw.set_finger_positions(cmd.fingers)

                # 节流更新状态面板
                if tick % TUI_UPDATE_EVERY_N_TICKS == 0:
                    live.update(
                        build_status_panel(
                            tick,
                            streamer.last_snap,
                            cmd.active,
                            to_sdk_positions(cmd.fingers),
                            T_target=cmd.T_target,
                            teleop_side=TELEOP_CONTROLLER_SIDE,
                        )
                    )

                tick += 1

                next_tick += LOOP_DT
                sleep_for = next_tick - time.perf_counter()
                if sleep_for > 0:
                    time.sleep(sleep_for)
                else:
                    next_tick = time.perf_counter()
    finally:
        log.info("正在释放硬件资源...")
        hw.disconnect()
        xr.close()
        log.info("已安全退出。")


if __name__ == "__main__":
    main()
