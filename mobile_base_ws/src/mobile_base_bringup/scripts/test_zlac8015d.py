#!/usr/bin/env python3
"""ZLAC8015D 单驱动器台架测试工具。

默认 status 子命令只读取 CANopen SDO，不使能电机。run 子命令必须带明确的
安全确认参数，并在正常退出、Ctrl+C 或异常时尝试发送零速和 Shutdown。
"""

import argparse
import math
import select
import socket
import struct
import sys
import time


CAN_FRAME = struct.Struct("=IB3x8s")


class SdoError(RuntimeError):
    """CANopen SDO 超时或拒绝请求。"""


class CanopenNode:
    def __init__(self, interface: str, node_id: int, timeout: float) -> None:
        self.node_id = node_id
        self.timeout = timeout
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))

    def close(self) -> None:
        self.sock.close()

    def _send(self, can_id: int, data: bytes) -> None:
        if len(data) > 8:
            raise ValueError("经典 CAN 数据不能超过 8 字节")
        self.sock.send(CAN_FRAME.pack(can_id, len(data), data.ljust(8, b"\x00")))

    def _exchange(self, request: bytes, index: int, subindex: int) -> bytes:
        self._send(0x600 + self.node_id, request)
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SdoError(
                    f"等待 node={self.node_id} 对象 0x{index:04X}:{subindex:02X} 回复超时"
                )
            readable, _, _ = select.select([self.sock], [], [], remaining)
            if not readable:
                continue
            raw = self.sock.recv(CAN_FRAME.size)
            can_id, length, payload = CAN_FRAME.unpack(raw)
            can_id &= 0x7FF
            if can_id != 0x580 + self.node_id or length < 8:
                continue
            reply_index = payload[1] | (payload[2] << 8)
            if reply_index != index or payload[3] != subindex:
                continue
            if payload[0] == 0x80:
                abort_code = int.from_bytes(payload[4:8], "little")
                raise SdoError(
                    f"对象 0x{index:04X}:{subindex:02X} 被拒绝，abort=0x{abort_code:08X}"
                )
            return payload

    def read(self, index: int, subindex: int = 0) -> int:
        request = bytes((0x40, index & 0xFF, index >> 8, subindex, 0, 0, 0, 0))
        reply = self._exchange(request, index, subindex)
        if reply[0] not in (0x4F, 0x4B, 0x47, 0x43):
            raise SdoError(f"对象 0x{index:04X}:{subindex:02X} 回复命令字异常：0x{reply[0]:02X}")
        byte_count = {0x4F: 1, 0x4B: 2, 0x47: 3, 0x43: 4}[reply[0]]
        return int.from_bytes(reply[4:4 + byte_count], "little", signed=False)

    def write(self, index: int, subindex: int, value: int, byte_count: int) -> None:
        command = {1: 0x2F, 2: 0x2B, 4: 0x23}[byte_count]
        raw_value = int(value).to_bytes(byte_count, "little", signed=True)
        request = bytes((command, index & 0xFF, index >> 8, subindex)) + raw_value
        request = request.ljust(8, b"\x00")
        reply = self._exchange(request, index, subindex)
        if reply[0] != 0x60:
            raise SdoError(f"写对象 0x{index:04X}:{subindex:02X} 未得到成功确认")

    def start_remote_node(self) -> None:
        # NMT 0x01 只让指定节点进入 Operational，不广播到其他节点。
        self._send(0x000, bytes((0x01, self.node_id)))


def signed32(value: int) -> int:
    return value - (1 << 32) if value & (1 << 31) else value


def read_status(node: CanopenNode, feedback_scale: float) -> None:
    mode = node.read(0x6061, 0) & 0xFF
    combined_status = node.read(0x6041, 0)
    fault_code = node.read(0x603F, 0)
    print(f"节点 {node.node_id} 通信正常")
    print(f"运行模式回读: {mode}（3 表示 Profile Velocity）")
    print(f"故障码: 0x{fault_code:08X}")
    for channel in (1, 2):
        statusword = (combined_status >> (16 * (channel - 1))) & 0xFFFF
        raw_velocity = signed32(node.read(0x606C, channel))
        rpm = raw_velocity * feedback_scale
        enabled = (statusword & 0x006F) == 0x0027
        print(
            f"通道 {channel}: statusword=0x{statusword:04X}, "
            f"enabled={'是' if enabled else '否'}, 实际转速={rpm:.2f} rpm"
        )


def stop_and_shutdown(node: CanopenNode) -> None:
    """无论测试如何结束，都尽量让两通道先归零再退出使能。"""
    for channel in (1, 2):
        try:
            node.write(0x60FF, channel, 0, 4)
        except Exception as error:  # 停机阶段继续尝试其他安全动作
            print(f"警告：通道 {channel} 零速指令失败：{error}", file=sys.stderr)
    time.sleep(0.2)
    try:
        node.write(0x6040, 0, 0x0006, 2)
    except Exception as error:
        print(f"警告：Shutdown 失败：{error}", file=sys.stderr)


def check_motion_safety(args: argparse.Namespace) -> None:
    if not args.yes_i_understand:
        raise RuntimeError("运动测试必须添加 --yes-i-understand，确认轮子已架空且可随时断电")
    if abs(args.rpm) > 100.0:
        raise RuntimeError("台架工具限制 |rpm| <= 100；首次建议使用 10 rpm")


def prepare_driver(node: CanopenNode, args: argparse.Namespace) -> None:
    """配置轮廓速度模式、缓加减速并使能两个通道。"""
    check_motion_safety(args)
    node.start_remote_node()
    time.sleep(0.05)

    # 使用异步轮廓速度模式；加减速时间设得较缓，降低首次测试冲击。
    node.write(0x200F, 0, 0, 2)
    node.write(0x6060, 0, 3, 1)
    if (node.read(0x6061, 0) & 0xFF) != 3:
        raise RuntimeError("驱动器未进入 Profile Velocity 模式")
    for channel in (1, 2):
        node.write(0x6083, channel, args.accel_ms, 4)
        node.write(0x6084, channel, args.decel_ms, 4)
        node.write(0x60FF, channel, 0, 4)

    # CiA-402：Shutdown → Switch On → Enable Operation。
    for controlword in (0x0006, 0x0007, 0x000F):
        node.write(0x6040, 0, controlword, 2)
        time.sleep(0.05)

    combined_status = node.read(0x6041, 0)
    for channel in (1, 2):
        statusword = (combined_status >> (16 * (channel - 1))) & 0xFFFF
        if (statusword & 0x006F) != 0x0027:
            raise RuntimeError(f"通道 {channel} 未进入 Operation Enabled：0x{statusword:04X}")


def set_rpm(node: CanopenNode, channel_rpm: dict[int, float], command_scale: float) -> None:
    """每次显式写两个通道，避免上一阶段的目标速度残留。"""
    for channel in (1, 2):
        raw_target = round(channel_rpm.get(channel, 0.0) * command_scale)
        node.write(0x60FF, channel, raw_target, 4)


def feedback_rpm(node: CanopenNode, channel: int, scale: float) -> float:
    return signed32(node.read(0x606C, channel)) * scale


def run_motor(node: CanopenNode, args: argparse.Namespace) -> None:
    if not 0.1 <= args.seconds <= 10.0:
        raise RuntimeError("测试时间必须在 0.1~10 秒之间")
    channels = (1, 2) if args.channel == "both" else (int(args.channel),)
    prepare_driver(node, args)

    print(f"开始：node={args.node}, channel={args.channel}, target={args.rpm:.2f} rpm")
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        set_rpm(node, {channel: args.rpm for channel in channels}, args.command_scale)
        time.sleep(0.1)

    for channel in channels:
        print(f"通道 {channel} 结束前反馈：{feedback_rpm(node, channel, args.feedback_scale):.2f} rpm")


def verify_driver(node: CanopenNode, args: argparse.Namespace) -> None:
    """依次验证两个通道正反转、共同转动和相反方向转动。"""
    if not 0.3 <= args.phase_seconds <= 3.0:
        raise RuntimeError("每阶段时间必须在 0.3~3 秒之间")
    if not 0.2 <= args.pause_seconds <= 3.0:
        raise RuntimeError("阶段间停车时间必须在 0.2~3 秒之间")
    prepare_driver(node, args)

    phases = (
        ("通道1正转", {1: args.rpm}),
        ("通道1反转", {1: -args.rpm}),
        ("通道2正转", {2: args.rpm}),
        ("通道2反转", {2: -args.rpm}),
        ("两通道同向", {1: args.rpm, 2: args.rpm}),
        ("两通道反向", {1: args.rpm, 2: -args.rpm}),
    )

    print("\n开始六阶段验证；每个阶段前均先发送双通道零速。")
    print("请观察对应轮子、机械方向、异响和驱动器报警灯。\n")
    for number, (name, targets) in enumerate(phases, start=1):
        set_rpm(node, {}, args.command_scale)
        time.sleep(args.pause_seconds)
        target_text = ", ".join(
            f"CH{channel}={rpm:+.1f} rpm" for channel, rpm in targets.items())
        print(f"[{number}/6] {name}，目标：{target_text}")

        deadline = time.monotonic() + args.phase_seconds
        samples = []
        while time.monotonic() < deadline:
            set_rpm(node, targets, args.command_scale)
            time.sleep(0.1)
            samples.append((
                feedback_rpm(node, 1, args.feedback_scale),
                feedback_rpm(node, 2, args.feedback_scale),
            ))
        ch1 = sum(sample[0] for sample in samples) / len(samples)
        ch2 = sum(sample[1] for sample in samples) / len(samples)
        print(f"      平均反馈：CH1={ch1:+.2f} rpm，CH2={ch2:+.2f} rpm")

    set_rpm(node, {}, args.command_scale)
    time.sleep(max(args.pause_seconds, 0.5))
    stopped_1 = feedback_rpm(node, 1, args.feedback_scale)
    stopped_2 = feedback_rpm(node, 2, args.feedback_scale)
    print(f"\n零速停车反馈：CH1={stopped_1:+.2f} rpm，CH2={stopped_2:+.2f} rpm")
    read_status(node, args.feedback_scale)
    print("\n自动流程完成；车辆安装后的前进方向仍需人工确认。")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ZLAC8015D 单驱动器、双通道安全台架测试")
    parser.add_argument("--interface", default="can0", help="SocketCAN 接口，默认 can0")
    parser.add_argument("--node", type=int, default=1, choices=range(1, 128), help="CANopen node ID")
    parser.add_argument("--timeout", type=float, default=0.5, help="单次 SDO 超时秒数")
    parser.add_argument("--feedback-scale", type=float, default=0.1, help="反馈原始值到 rpm 的倍率")
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("status", help="只读状态，不使能、不转动")
    run = subparsers.add_parser("run", help="低速运行指定通道")
    run.add_argument("--channel", choices=("1", "2", "both"), required=True)
    run.add_argument("--rpm", type=float, default=10.0)
    run.add_argument("--seconds", type=float, default=2.0)
    run.add_argument("--accel-ms", type=int, default=1000)
    run.add_argument("--decel-ms", type=int, default=1000)
    run.add_argument("--command-scale", type=float, default=1.0, help="rpm 到指令原始值的倍率")
    run.add_argument("--yes-i-understand", action="store_true", help="确认轮子已架空并可立即断电")

    verify = subparsers.add_parser("verify", help="自动执行双通道六阶段正反转验证")
    verify.add_argument("--rpm", type=float, default=10.0)
    verify.add_argument("--phase-seconds", type=float, default=1.0, help="每个运动阶段持续时间")
    verify.add_argument("--pause-seconds", type=float, default=0.7, help="阶段间零速等待时间")
    verify.add_argument("--accel-ms", type=int, default=1000)
    verify.add_argument("--decel-ms", type=int, default=1000)
    verify.add_argument("--command-scale", type=float, default=1.0, help="rpm 到指令原始值的倍率")
    verify.add_argument("--yes-i-understand", action="store_true", help="确认轮子已架空并可立即断电")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    node = None
    try:
        node = CanopenNode(args.interface, args.node, args.timeout)
        if args.command == "status":
            read_status(node, args.feedback_scale)
        else:
            try:
                if args.command == "run":
                    run_motor(node, args)
                else:
                    verify_driver(node, args)
            finally:
                stop_and_shutdown(node)
                print("已发送两通道零速和 Shutdown")
        return 0
    except (OSError, SdoError, RuntimeError, ValueError) as error:
        print(f"测试失败：{error}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n收到 Ctrl+C，正在安全停机……", file=sys.stderr)
        return 130
    finally:
        if node is not None:
            node.close()


if __name__ == "__main__":
    raise SystemExit(main())
