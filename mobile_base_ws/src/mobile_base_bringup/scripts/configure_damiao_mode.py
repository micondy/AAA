#!/usr/bin/env python3
"""安全读取/切换达妙 CTRL_MODE；不会发送运动或设零指令。"""
import argparse
import os
import select
import socket
import struct
import sys
import time
from pathlib import Path

import yaml

CAN_FRAME = struct.Struct("=IB3x8s")
REQUEST_ID, READ, WRITE, SAVE = 0x7FF, 0x33, 0x55, 0xAA
CTRL_MODE, DISABLE = 0x0A, 0xFD
WHEELS = ("front_left", "front_right", "rear_left", "rear_right")
MODES = {"mit": 1, "pos_vel": 2, "vel": 3}
MODE_NAMES = {value: name for name, value in MODES.items()}


def integer(value):
    return int(value, 0) if isinstance(value, str) else int(value)


def default_config_path():
    directory = os.environ.get("MOBILE_BASE_CONFIG_DIR")
    root = Path(directory).expanduser() if directory else Path.home() / ".config/mobile_base"
    return root / "chassis.yaml"


def load_motors(path):
    if not path.is_file():
        raise FileNotFoundError(f"找不到 {path}；请用 --config 指定 chassis.yaml")
    config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if set(config.get("wheels", {})) != set(WHEELS):
        raise ValueError("配置必须包含四个标准轮组")
    motors = {}
    for wheel in WHEELS:
        steering = config["wheels"][wheel]["steering"]
        motor_id = integer(steering["motor_id"])
        master_id = integer(steering["master_id"])
        if not 1 <= motor_id <= 0x0F or not 0 <= master_id <= 0x7FF:
            raise ValueError(f"{wheel} 的 ESC_ID/MST_ID 超出范围")
        motors[wheel] = motor_id, master_id
    return motors, config["can"]["steering"]["interface"]


def ros2_control_is_running():
    """禁止和 ros2_control 同时写同一条 CAN 总线。"""
    for entry in Path("/proc").iterdir():
        if not entry.name.isdigit():
            continue
        try:
            command = (entry / "cmdline").read_bytes().replace(b"\0", b" ")
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if b"controller_manager/ros2_control_node" in command:
            return True
    return False


class DamiaoModeConfigurator:
    def __init__(self, interface, timeout):
        self.timeout = timeout
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))

    def close(self):
        self.sock.close()

    def send(self, can_id, payload):
        self.sock.send(CAN_FRAME.pack(can_id, 8, payload))

    def wait_parameter(self, motor_id, master_id, command):
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"等待 ESC=0x{motor_id:02X}、MST=0x{master_id:03X} 参数回复超时"
                )
            readable, _, _ = select.select([self.sock], [], [], remaining)
            if not readable:
                continue
            can_id, length, payload = CAN_FRAME.unpack(self.sock.recv(CAN_FRAME.size))
            reply_motor = payload[0] | (payload[1] << 8)
            if (
                (can_id & 0x7FF) == master_id and length >= 8
                and reply_motor == motor_id and payload[2] == command
                and payload[3] == CTRL_MODE
            ):
                return struct.unpack("<I", payload[4:8])[0]

    def read_mode(self, motor_id, master_id):
        self.send(REQUEST_ID, bytes((motor_id, 0, READ, CTRL_MODE, 0, 0, 0, 0)))
        return self.wait_parameter(motor_id, master_id, READ)

    def disable(self, motor_id):
        self.send(motor_id, b"\xFF\xFF\xFF\xFF\xFF\xFF\xFF" + bytes((DISABLE,)))
        time.sleep(0.02)

    def set_mode(self, motor_id, master_id, mode):
        self.disable(motor_id)
        payload = bytes((motor_id, 0, WRITE, CTRL_MODE)) + struct.pack("<I", mode)
        self.send(REQUEST_ID, payload)
        reply = self.wait_parameter(motor_id, master_id, WRITE)
        if reply != mode:
            raise RuntimeError(f"写入回包为 {reply}，与目标 {mode} 不一致")
        verified = self.read_mode(motor_id, master_id)
        if verified != mode:
            raise RuntimeError(f"CTRL_MODE 回读为 {verified}，与目标 {mode} 不一致")

    def save(self, motor_id):
        self.disable(motor_id)
        self.send(REQUEST_ID, bytes((motor_id, 0, SAVE, 0, 0, 0, 0, 0)))
        time.sleep(0.1)


def mode_text(mode):
    return f"{mode} ({MODE_NAMES.get(mode, '未知模式')})"


def main():
    parser = argparse.ArgumentParser(description="达妙 CTRL_MODE 读取与切换")
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--interface", help="覆盖配置中的转向 CAN 口")
    parser.add_argument("--timeout", type=float, default=0.5)
    commands = parser.add_subparsers(dest="command", required=True)
    read = commands.add_parser("read", help="只读 CTRL_MODE")
    read.add_argument("--wheel", choices=("all",) + WHEELS, default="all")
    read.add_argument(
        "--require-mode", choices=tuple(MODES),
        help="若任一已读取电机不是指定模式，则以失败状态退出")
    write = commands.add_parser("set", help="失能后切换 CTRL_MODE")
    write.add_argument("--wheel", choices=("all",) + WHEELS, required=True)
    write.add_argument("--mode", choices=tuple(MODES), required=True)
    write.add_argument("--persist", action="store_true", help="保存到电机 Flash")
    write.add_argument("--yes-i-understand", action="store_true")
    write.add_argument("--yes-save-to-flash", action="store_true")
    args = parser.parse_args()
    if not 0.05 <= args.timeout <= 5.0:
        parser.error("--timeout 必须在 0.05 到 5 秒之间")

    motors, configured_interface = load_motors(args.config)
    interface = args.interface or configured_interface
    names = WHEELS if args.wheel == "all" else (args.wheel,)
    if args.command == "set":
        if not args.yes_i_understand:
            raise RuntimeError("切换模式前必须添加 --yes-i-understand")
        if args.persist and not args.yes_save_to_flash:
            raise RuntimeError("保存 Flash 前必须额外添加 --yes-save-to-flash")
        if ros2_control_is_running():
            raise RuntimeError("检测到 ros2_control_node；请先 Ctrl+C 停止底盘 launch")

    tool = DamiaoModeConfigurator(interface, args.timeout)
    try:
        for name in names:
            motor_id, master_id = motors[name]
            if args.command == "read":
                mode = tool.read_mode(motor_id, master_id)
                print(
                    f"{name}: ESC=0x{motor_id:02X}, MST=0x{master_id:03X}, "
                    f"CTRL_MODE={mode_text(mode)}"
                )
                if args.require_mode and mode != MODES[args.require_mode]:
                    raise RuntimeError(
                        f"{name} 当前为 {mode_text(mode)}，要求 {args.require_mode}")
                continue
            target = MODES[args.mode]
            before = tool.read_mode(motor_id, master_id)
            print(f"{name}: CTRL_MODE {mode_text(before)} -> {mode_text(target)}")
            tool.set_mode(motor_id, master_id, target)
            if args.persist:
                tool.save(motor_id)
                print("  已请求保存 Flash；断电重启后请再次 read 验证")
            else:
                print("  RAM 切换成功；断电后恢复原模式")
    finally:
        tool.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("已取消；未发送运动指令", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print(f"错误：{error}", file=sys.stderr)
        raise SystemExit(1)
