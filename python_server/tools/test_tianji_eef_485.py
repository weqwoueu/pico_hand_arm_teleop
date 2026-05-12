"""天机末端 8Pin 485 透传诊断脚本。

目标：一次运行尽量判断问题在机器人连接、末端通道、从站号、A/B 线序、
波特率/串口参数，还是 Revo2 协议帧。

链路应为：

    PC 网口 -> 天机控制器 SDK -> A/B 臂末端 COM1/COM2 -> 8Pin 485 -> Revo2

通道约定来自天机 SDK：

    channel=1  CAN/CANFD
    channel=2  COM1，通常对应末端 8Pin 485 通道 1
    channel=3  COM2，通常对应末端 8Pin 485 通道 2

用法（在 python_server 目录）::

    ./run.sh -m tools.test_tianji_eef_485
    ./run.sh -m tools.test_tianji_eef_485 --arm A --channels 2 3 --slaves 0x7e 0x7f 1
    ./run.sh -m tools.test_tianji_eef_485 --ip 192.168.71.190 --channel 2 --slave 0x7e

默认只发 Modbus RTU 读请求，读请求本身不会改变 Revo2 状态。
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from dataclasses import dataclass
from pathlib import Path

# 兼容两种启动方式：
#   1) ./run.sh -m tools.test_tianji_eef_485
#   2) python tools/test_tianji_eef_485.py
_PKG_ROOT = Path(__file__).resolve().parent.parent
if str(_PKG_ROOT) not in sys.path:
    sys.path.insert(0, str(_PKG_ROOT))

from core.arm_core import ensure_tianji_sdk_path  # noqa: E402

ensure_tianji_sdk_path()
from SDK_PYTHON.fx_robot import DCSS, Marvin_Robot  # type: ignore  # noqa: E402

logger = logging.getLogger("test_tianji_eef_485")


CHANNEL_NAMES = {
    1: "CAN/CANFD",
    2: "COM1 / 8Pin 485 通道1",
    3: "COM2 / 8Pin 485 通道2",
}


@dataclass
class ProbeResult:
    channel: int
    slave: int
    request: bytes
    sent_ok: bool
    raw_reply: bytes
    raw_len: int
    note: str


def _parse_int_auto(raw: str) -> int:
    try:
        value = int(str(raw), 0)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"无法解析整数: {raw}") from exc
    return value


def _hex_bytes(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


def _parse_hex_payload(raw: str) -> bytes:
    text = raw.replace(",", " ").replace("0x", "").replace("0X", "")
    parts = [p for p in text.split() if p]
    if not parts:
        raise argparse.ArgumentTypeError("HEX 数据不能为空")
    try:
        return bytes(int(p, 16) for p in parts)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"HEX 数据格式错误: {raw}") from exc


def _crc16_modbus(payload: bytes) -> int:
    crc = 0xFFFF
    for b in payload:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def _with_crc(payload: bytes) -> bytes:
    crc = _crc16_modbus(payload)
    return payload + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def _read_holding_frame(slave: int, reg: int, count: int) -> bytes:
    return _with_crc(
        bytes(
            (
                slave & 0xFF,
                0x03,
                (reg >> 8) & 0xFF,
                reg & 0xFF,
                (count >> 8) & 0xFF,
                count & 0xFF,
            )
        )
    )


def _read_input_frame(slave: int, reg: int, count: int) -> bytes:
    return _with_crc(
        bytes(
            (
                slave & 0xFF,
                0x04,
                (reg >> 8) & 0xFF,
                reg & 0xFF,
                (count >> 8) & 0xFF,
                count & 0xFF,
            )
        )
    )


def _write_single_frame(slave: int, reg: int, value: int) -> bytes:
    return _with_crc(
        bytes(
            (
                slave & 0xFF,
                0x06,
                (reg >> 8) & 0xFF,
                reg & 0xFF,
                (value >> 8) & 0xFF,
                value & 0xFF,
            )
        )
    )


def _reply_from_sdk(raw_hex: object, read_len: int) -> bytes:
    if read_len <= 0 or raw_hex is None:
        return b""
    if isinstance(raw_hex, bytes):
        return raw_hex[:read_len]
    text = str(raw_hex).strip()
    if not text:
        return b""
    out: list[int] = []
    for part in text.split():
        try:
            out.append(int(part, 16))
        except ValueError:
            break
        if len(out) >= read_len:
            break
    return bytes(out)


def _classify_reply(reply: bytes, slave: int, request: bytes) -> str:
    if not reply:
        return "无回包"
    if len(reply) < 3:
        return "收到短包：物理链路可能有数据，但帧不完整，疑似波特率/串口参数不匹配或干扰"
    if reply[0] != (slave & 0xFF):
        return f"收到其他从站/噪声: first=0x{reply[0]:02X}，目标 slave=0x{slave:02X}"
    if len(reply) >= 5:
        body, crc_lo, crc_hi = reply[:-2], reply[-2], reply[-1]
        crc = _crc16_modbus(body)
        if crc_lo != (crc & 0xFF) or crc_hi != ((crc >> 8) & 0xFF):
            return "收到目标从站数据，但 CRC 错：疑似波特率/串口参数不匹配、A/B 接触不稳或线路干扰"
    fn = reply[1]
    req_fn = request[1] if len(request) > 1 else None
    if req_fn is not None and fn == (req_fn | 0x80):
        code = reply[2] if len(reply) > 2 else -1
        return f"Modbus 异常回包 code=0x{code:02X}：物理链路/通道/波特率/从站号大概率已通，寄存器或功能码不匹配"
    if req_fn is not None and fn == req_fn:
        return "正常 Modbus 回包：8Pin 485 透传链路已通"
    return f"收到目标从站回包，但功能码异常 fn=0x{fn:02X}，请看 raw"


def _drain(robot: Marvin_Robot, arm: str, channel: int, seconds: float) -> list[bytes]:
    deadline = time.perf_counter() + max(0.0, seconds)
    replies: list[bytes] = []
    while time.perf_counter() < deadline:
        read_len, raw = robot.get_485_data(arm, channel)
        if read_len and read_len > 0:
            replies.append(_reply_from_sdk(raw, int(read_len)))
        else:
            time.sleep(0.01)
    return replies


def _send_and_read(
    robot: Marvin_Robot,
    *,
    arm: str,
    channel: int,
    slave: int,
    frame: bytes,
    read_wait_s: float,
    read_loops: int,
) -> ProbeResult:
    ok, sdk_ret = robot.set_485_data(arm, frame, len(frame), channel)
    sent_ok = bool(ok and sdk_ret)
    raw_reply = b""
    raw_len = 0
    note = "发送失败"
    if sent_ok:
        time.sleep(read_wait_s)
        for _ in range(max(1, read_loops)):
            read_len, raw = robot.get_485_data(arm, channel)
            if read_len and read_len > 0:
                raw_len = int(read_len)
                raw_reply = _reply_from_sdk(raw, raw_len)
                break
            time.sleep(read_wait_s)
        note = _classify_reply(raw_reply, slave, frame)
    return ProbeResult(channel, slave, frame, sent_ok, raw_reply, raw_len, note)


def _verify_robot_link(robot: Marvin_Robot, dcss: DCSS, tries: int = 8) -> None:
    motion_tag = 0
    prev = None
    for _ in range(max(1, tries)):
        sub = robot.subscribe(dcss)
        fs = sub["outputs"][0]["frame_serial"]
        logger.info("connect frame_serial=%s", fs)
        if fs != 0 and fs != prev:
            motion_tag += 1
            prev = fs
        time.sleep(0.02)
    if motion_tag <= 0:
        raise RuntimeError("未检测到 frame_serial 刷新：先查网线/IP/防火墙/控制器连接")


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="天机末端 8Pin 485 透传诊断")
    ap.add_argument(
        "--ip",
        default=os.environ.get("MARVIN_IP", "192.168.1.190"),
        help="天机控制器 IP，也可用 MARVIN_IP",
    )
    ap.add_argument("--arm", choices=("A", "B"), default="A", help="末端所在手臂，默认 A")
    ap.add_argument(
        "--channels",
        type=_parse_int_auto,
        nargs="+",
        default=[2, 3],
        help="要扫描的末端通道：2=COM1/485通道1，3=COM2/485通道2；默认 2 3",
    )
    ap.add_argument(
        "--channel",
        type=_parse_int_auto,
        default=None,
        help="只测一个通道，等价于 --channels <channel>",
    )
    ap.add_argument(
        "--slaves",
        type=_parse_int_auto,
        nargs="+",
        default=[0x7E, 0x7F, 0x01],
        help="要扫描的 Modbus 从站号，默认 0x7e 0x7f 1",
    )
    ap.add_argument("--slave", type=_parse_int_auto, default=None, help="只测一个从站号")
    ap.add_argument(
        "--read-regs",
        type=_parse_int_auto,
        nargs="+",
        default=[0x0000, 0x0001, 0x0100],
        help="读寄存器探测地址，默认 0x0000 0x0001 0x0100",
    )
    ap.add_argument("--read-count", type=_parse_int_auto, default=1, help="每次读几个寄存器，默认 1")
    ap.add_argument(
        "--functions",
        choices=("03", "04", "both"),
        default="both",
        help="读 holding(03)、input(04) 或二者都试，默认 both",
    )
    ap.add_argument(
        "--custom-hex",
        type=_parse_hex_payload,
        default=None,
        help="自定义完整 Modbus RTU 帧 HEX；不自动补 CRC，例如 '7F 03 00 00 00 01 XX XX'",
    )
    ap.add_argument("--read-wait-ms", type=float, default=80.0, help="发出后每次等待回包 ms，默认 80")
    ap.add_argument("--read-loops", type=int, default=8, help="每个请求最多读几次，默认 8")
    ap.add_argument("--pre-drain-ms", type=float, default=150.0, help="每个通道测试前先清/读缓存 ms，默认 150")
    ap.add_argument("--sdk-log", action="store_true", help="打开天机 SDK 日志")
    ap.add_argument(
        "--write-open",
        action="store_true",
        help="额外发送 function 06 写 0 到寄存器 0 的测试帧；可能改变设备状态，默认不发",
    )
    return ap


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")
    args = build_arg_parser().parse_args()

    channels = [args.channel] if args.channel is not None else args.channels
    slaves = [args.slave] if args.slave is not None else args.slaves
    channels = [int(ch) for ch in channels]
    slaves = [int(sl) for sl in slaves]
    for ch in channels:
        if ch not in (2, 3):
            raise ValueError("本脚本只诊断末端 485，通道请用 2(COM1) 或 3(COM2)")
    for sl in slaves:
        if not 0 <= sl <= 247:
            raise ValueError(f"Modbus 从站号应在 0~247: {sl}")

    read_wait_s = max(0.0, args.read_wait_ms) * 0.001
    pre_drain_s = max(0.0, args.pre_drain_ms) * 0.001

    logger.info("连接天机控制器 ip=%s arm=%s", args.ip, args.arm)
    robot = Marvin_Robot()
    dcss = DCSS()
    if robot.connect(args.ip) != 1:
        raise RuntimeError("连接天机控制器失败：检查 IP、网线或端口占用")

    any_reply = False
    try:
        robot.check_error_and_clear(dcss)
        _verify_robot_link(robot, dcss)
        if args.sdk_log:
            robot.log_switch("1")
            robot.local_log_switch("1")
        else:
            robot.log_switch("0")
            robot.local_log_switch("0")

        logger.info(
            "开始诊断：channels=%s slaves=%s read_regs=%s functions=%s",
            channels,
            [f"0x{x:02X}" for x in slaves],
            [f"0x{x:04X}" for x in args.read_regs],
            args.functions,
        )

        for channel in channels:
            logger.info("==== 测试 channel=%d (%s) ====", channel, CHANNEL_NAMES[channel])
            clear_ok = robot.clear_485_cache(args.arm)
            logger.info("clear_485_cache(%s) -> %s", args.arm, clear_ok)
            stale = _drain(robot, args.arm, channel, pre_drain_s)
            if stale:
                logger.info(
                    "清缓存后仍读到旧数据 %d 包：%s",
                    len(stale),
                    [" ".join(f"{b:02X}" for b in p[:32]) for p in stale[:3]],
                )

            for slave in slaves:
                frames: list[tuple[str, bytes]] = []
                if args.custom_hex is not None:
                    frames.append(("custom", args.custom_hex))
                else:
                    for reg in args.read_regs:
                        if args.functions in {"03", "both"}:
                            frames.append((f"read03 reg=0x{reg:04X}", _read_holding_frame(slave, reg, args.read_count)))
                        if args.functions in {"04", "both"}:
                            frames.append((f"read04 reg=0x{reg:04X}", _read_input_frame(slave, reg, args.read_count)))
                    if args.write_open:
                        frames.append(("write06 reg=0x0000 value=0", _write_single_frame(slave, 0x0000, 0)))

                for label, frame in frames:
                    result = _send_and_read(
                        robot,
                        arm=args.arm,
                        channel=channel,
                        slave=slave,
                        frame=frame,
                        read_wait_s=read_wait_s,
                        read_loops=args.read_loops,
                    )
                    if result.raw_reply:
                        any_reply = True
                    logger.info(
                        "ch=%d slave=0x%02X %-24s sent=%s req=[%s] reply_len=%d reply=[%s] -> %s",
                        result.channel,
                        result.slave,
                        label,
                        result.sent_ok,
                        _hex_bytes(result.request),
                        result.raw_len,
                        _hex_bytes(result.raw_reply),
                        result.note,
                    )

        logger.info("==== 诊断结论 ====")
        if any_reply:
            logger.info("收到过回包：8Pin 485 物理链路至少有一路可达。优先根据正常/异常回包调整协议帧。")
        else:
            logger.info("所有探测均无回包。优先按以下顺序查：")
            logger.info("1. 8Pin 通道是否接对：NO.3/4 对 channel=2，NO.5/6 对 channel=3")
            logger.info("2. 485 A/B 是否反了：交换 A/B 再跑一次")
            logger.info("3. Revo2 从站号是否不是 0x7E/0x7F/0x01：用 --slaves 扩大扫描")
            logger.info("4. 末端 COM 波特率/校验位/停止位与 Revo2 不一致：这是最像的波特率问题")
            logger.info("5. 末端 485 通道未启用或需要控制器侧配置：问天机 COM1/COM2 串口参数/启用方式")
    finally:
        robot.release_robot()
        logger.info("已释放天机 SDK 连接。")


if __name__ == "__main__":
    main()
