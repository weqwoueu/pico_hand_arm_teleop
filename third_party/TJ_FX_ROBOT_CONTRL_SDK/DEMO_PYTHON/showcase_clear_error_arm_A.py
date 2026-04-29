import argparse
import logging
import os
import sys
import time

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir = os.path.dirname(current_dir)
sys.path.insert(0, parent_dir)

from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot, arm_err_code  # noqa: E402

'''#################################################################
A 臂专用：清错 + 下使能 + 再尝试进位置模式（state=1）

背景
- cur_state=100 表示臂级错误，需清错后再使能/切模式（README）。
- err_code=6 为「请求进扭矩失败」，常与硬接触、工具动力学、急停/初始化等有关；
  软件侧必须保证：clear_error 与 set_state 走 clear_set/send_cmd 批次下发。
- 内置 check_error_and_clear() 对双臂用 elif，且 clear_error 后若不 send_cmd，
  部分现场会出现「清不掉」的假象。

用法
  export MARVIN_IP=192.168.71.190   # 可选，默认与下面 --ip 一致
  python3 showcase_clear_error_arm_A.py
  python3 showcase_clear_error_arm_A.py --ip 192.168.71.190 --cycles 8 --try-position

若多轮后仍为 100/6：按 README 查硬件接触、工具动力学、单臂是否误控 A、断电重启控制柜。
'''#################################################################

logger = logging.getLogger("clear_arm_a")
logging.basicConfig(format="%(message)s", level=logging.INFO)


def _err_hint(code: int) -> str:
    return arm_err_code.get(str(int(code)), "(无字典条目，见 README 臂级错误表)")


def _read_arm_a(robot: Marvin_Robot, dcss: DCSS):
    sub = robot.subscribe(dcss)
    st = sub["states"][0]
    return st["cur_state"], st["err_code"], st.get("cmd_state", -1), sub


def _pulse_clear_a(robot: Marvin_Robot) -> None:
    robot.clear_set()
    robot.clear_error("A")
    robot.send_cmd()


def _arm_a_down(robot: Marvin_Robot) -> None:
    robot.clear_set()
    robot.set_state(arm="A", state=0)
    robot.send_cmd()


def _arm_a_position_idle(robot: Marvin_Robot, vel: int, acc: int) -> None:
    robot.clear_set()
    robot.set_state(arm="A", state=1)
    robot.set_vel_acc(arm="A", velRatio=vel, AccRatio=acc)
    robot.send_cmd()


def main() -> int:
    p = argparse.ArgumentParser(description="A 臂：下使能 + 清错脉冲 + 可选进位置")
    p.add_argument(
        "--ip",
        default=os.environ.get("MARVIN_IP", "192.168.71.190"),
        help="控制器 IP（也可用环境变量 MARVIN_IP）",
    )
    p.add_argument(
        "--cycles",
        type=int,
        default=10,
        help="下使能+清错脉冲最大循环次数",
    )
    p.add_argument(
        "--try-position",
        action="store_true",
        help="清错循环结束后尝试 set_state(A,1)+set_vel_acc 并轮询是否脱离 100",
    )
    p.add_argument(
        "--vel",
        type=int,
        default=10,
        help="进位置时的速度百分比 0~100",
    )
    p.add_argument(
        "--acc",
        type=int,
        default=10,
        help="进位置时的加速度百分比 0~100",
    )
    p.add_argument(
        "--also-clear-b-once",
        action="store_true",
        help="连接成功后额外对 B 清错一次（共柜总线异常时可试）",
    )
    args = p.parse_args()

    dcss = DCSS()
    robot = Marvin_Robot()

    if robot.connect(args.ip) == 0:
        logger.error("连接失败（端口占用或未连通）")
        return 1

    time.sleep(0.5)

    motion_tag = 0
    frame_prev = None
    for _ in range(5):
        sub = robot.subscribe(dcss)
        fs = sub["outputs"][0]["frame_serial"]
        if fs != 0 and fs != frame_prev:
            motion_tag += 1
            frame_prev = fs
        time.sleep(0.02)
    if motion_tag == 0:
        logger.error("未检测到 frame_serial 刷新，请查防火墙/网线/独占连接")
        robot.release_robot()
        return 1

    robot.log_switch("0")
    robot.local_log_switch("0")

    # 与 joint_drag 一致：清错必须跟 send_cmd
    robot.clear_set()
    robot.clear_error("A")
    robot.send_cmd()
    time.sleep(0.35)
    if args.also_clear_b_once:
        robot.clear_set()
        robot.clear_error("B")
        robot.send_cmd()
        time.sleep(0.35)

    cur, err, cmd, _ = _read_arm_a(robot, dcss)
    logger.info(
        f"初始 A: cur_state={cur} cmd_state={cmd} err_code={err} → {_err_hint(err)}"
    )

    for k in range(args.cycles):
        cur, err, cmd, _ = _read_arm_a(robot, dcss)
        if cur != 100 and err == 0:
            logger.info(f"第 {k} 轮前已正常: cur_state={cur} err_code={err}")
            break
        logger.info(
            f"第 {k + 1}/{args.cycles} 轮: A cur_state={cur} cmd_state={cmd} err_code={err}"
        )
        _arm_a_down(robot)
        time.sleep(0.4)
        _pulse_clear_a(robot)
        time.sleep(0.55)
        _pulse_clear_a(robot)
        time.sleep(0.35)
    else:
        cur, err, cmd, _ = _read_arm_a(robot, dcss)

    cur, err, cmd, _ = _read_arm_a(robot, dcss)
    logger.info(
        f"清错循环结束 A: cur_state={cur} cmd_state={cmd} err_code={err} → {_err_hint(err)}"
    )

    if args.try_position:
        _arm_a_position_idle(robot, args.vel, args.acc)
        time.sleep(0.85)
        for j in range(15):
            cur, err, cmd, _ = _read_arm_a(robot, dcss)
            logger.info(
                f"进位置后采样 {j + 1}/15: cur_state={cur} cmd_state={cmd} err_code={err}"
            )
            if cur == 1 and err == 0:
                logger.info("A 臂已进入位置跟随且 err_code=0")
                robot.clear_set()
                robot.set_state(arm="A", state=0)
                robot.send_cmd()
                robot.release_robot()
                return 0
            if cur == 100 or err != 0:
                _arm_a_down(robot)
                time.sleep(0.35)
                _pulse_clear_a(robot)
                time.sleep(0.5)
                _arm_a_position_idle(robot, args.vel, args.acc)
                time.sleep(0.85)
            time.sleep(0.08)

    cur, err, cmd, _ = _read_arm_a(robot, dcss)
    if cur == 100 or err != 0:
        logger.warning(
            "仍未恢复。若软件侧已尽力：检查急停/硬接触/工具动力学/INI 单双臂配置；"
            "或断电重启控制柜与驱动（README）。"
        )
        logger.warning("可再试: python3 showcase_clear_error_arm_A.py --also-clear-b-once --try-position")

    robot.clear_set()
    robot.set_state(arm="A", state=0)
    robot.send_cmd()
    robot.release_robot()
    return 0 if (cur != 100 and err == 0) else 2


if __name__ == "__main__":
    sys.exit(main())
