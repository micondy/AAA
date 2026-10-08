#!/usr/bin/env python3
"""达妙转向绝对零位读取与标定工具。

只读取输出轴绝对位置寄存器 0x51，并在明确确认后修改 chassis.yaml 的
offset_rad。不会使能电机、不会发送位置命令，也不会发送电机内部设零命令。
"""

import argparse
import datetime
import math
import os
import select
import socket
import struct
import sys
import time
from pathlib import Path

import yaml

CAN_FRAME = struct.Struct("=IB3x8s")
PARAMETER_REQUEST_ID = 0x7FF
PARAMETER_READ = 0x33
ABSOLUTE_POSITION_REGISTER = 0x51
WHEEL_NAMES = ("front_left", "front_right", "rear_left", "rear_right")


def integer(value):
    return int(value, 0) if isinstance(value, str) else int(value)


class DamiaoParameterReader:
    """通过经典 SocketCAN 读取达妙参数，不包含任何运动指令。"""

    def __init__(self, interface: str, timeout: float):
        self.timeout = timeout
        self.sock = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        self.sock.bind((interface,))

    def close(self):
        self.sock.close()

    def read_float(self, motor_id: int, master_id: int, register: int) -> float:
        request = bytes((motor_id, 0, PARAMETER_READ, register, 0, 0, 0, 0))
        self.sock.send(CAN_FRAME.pack(PARAMETER_REQUEST_ID, 8, request))
        deadline = time.monotonic() + self.timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError(
                    f"等待 ESC_ID=0x{motor_id:02X}、MST_ID=0x{master_id:03X} 回复超时"
                )
            readable, _, _ = select.select([self.sock], [], [], remaining)
            if not readable:
                continue
            raw = self.sock.recv(CAN_FRAME.size)
            can_id, length, payload = CAN_FRAME.unpack(raw)
            reply_motor = payload[0] | (payload[1] << 8)
            if (
                (can_id & 0x7FF) != master_id or length < 8 or
                reply_motor != motor_id or payload[2] != PARAMETER_READ or
                payload[3] != register
            ):
                continue
            value = struct.unpack("<f", payload[4:8])[0]
            if not math.isfinite(value):
                raise RuntimeError("电机返回了非有限绝对位置")
            return float(value)


def default_config_path() -> Path:
    config_dir = os.environ.get("MOBILE_BASE_CONFIG_DIR")
    if config_dir:
        return Path(config_dir).expanduser() / "chassis.yaml"
    return Path.home() / ".config" / "mobile_base" / "chassis.yaml"


def load_config(path: Path):
    if not path.is_file():
        raise FileNotFoundError(
            f"找不到 {path}；请先在网页保存底盘配置，或用 --config 指定文件"
        )
    config = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if set(config.get("wheels", {})) != set(WHEEL_NAMES):
        raise ValueError("配置必须包含四个标准轮组")
    return config


def read_wheel(reader, config, wheel_name):
    steering = config["wheels"][wheel_name]["steering"]
    motor_id, master_id = integer(steering["motor_id"]), integer(steering["master_id"])
    raw_position = reader.read_float(motor_id, master_id, ABSOLUTE_POSITION_REGISTER)
    direction = int(steering["direction"])
    offset = float(steering["offset_rad"])
    return {
        "motor_id": motor_id,
        "master_id": master_id,
        "raw_position": raw_position,
        "direction": direction,
        "offset": offset,
        "joint_position": raw_position * direction + offset,
    }


def save_offset(path: Path, config, wheel_name: str, offset: float) -> Path:
    backup_dir = path.parent / "backup"
    backup_dir.mkdir(parents=True, exist_ok=True)
    stamp = datetime.datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    backup = backup_dir / f"chassis-before-zero-{stamp}.yaml"
    backup.write_bytes(path.read_bytes())
    config["wheels"][wheel_name]["steering"]["offset_rad"] = float(offset)
    temporary = path.with_suffix(".yaml.tmp")
    temporary.write_text(
        yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8"
    )
    temporary.replace(path)
    return backup


def main():
    parser = argparse.ArgumentParser(description="达妙转向绝对零位读取与标定")
    parser.add_argument("--config", type=Path, default=default_config_path())
    parser.add_argument("--interface", help="覆盖 chassis.yaml 中的转向 CAN 口")
    parser.add_argument("--timeout", type=float, default=0.5)
    commands = parser.add_subparsers(dest="command", required=True)
    read_parser = commands.add_parser("read", help="只读一个或全部转向电机")
    read_parser.add_argument("--wheel", choices=("all",) + WHEEL_NAMES, default="all")
    calibrate = commands.add_parser("calibrate", help="把当前机械朝前位置标为 0 rad")
    calibrate.add_argument("--wheel", choices=WHEEL_NAMES, required=True)
    calibrate.add_argument("--write", action="store_true", help="确认后写入 chassis.yaml")
    calibrate.add_argument("--yes-wheel-is-straight", action="store_true")
    args = parser.parse_args()
    if not 0.05 <= args.timeout <= 5.0:
        parser.error("--timeout 必须在 0.05 到 5 秒之间")

    config = load_config(args.config)
    interface = args.interface or config["can"]["steering"]["interface"]
    reader = DamiaoParameterReader(interface, args.timeout)
    try:
        if args.command == "read":
            names = WHEEL_NAMES if args.wheel == "all" else (args.wheel,)
            for name in names:
                value = read_wheel(reader, config, name)
                print(
                    f"{name}: ESC=0x{value['motor_id']:02X}, MST=0x{value['master_id']:03X}, "
                    f"绝对位置={value['raw_position']:+.6f} rad, "
                    f"当前关节角={value['joint_position']:+.6f} rad"
                )
            return

        value = read_wheel(reader, config, args.wheel)
        new_offset = -value["raw_position"] * value["direction"]
        print(f"轮组：{args.wheel}")
        print(f"输出轴绝对位置：{value['raw_position']:+.6f} rad")
        print(f"方向：{value['direction']:+d}")
        print(f"旧 offset_rad：{value['offset']:+.6f}")
        print(f"建议新 offset_rad：{new_offset:+.6f}")
        if not args.write:
            print("仅预览，未修改配置。确认机械朝前后添加 --write --yes-wheel-is-straight")
            return
        if not args.yes_wheel_is_straight:
            raise RuntimeError("写入前必须添加 --yes-wheel-is-straight，确认该轮已机械对正")
        backup = save_offset(args.config, config, args.wheel, new_offset)
        print(f"标定已保存：{args.config}")
        print(f"原配置备份：{backup}")
        print("请重启底盘，并以低速命令到 0° 验证方向和零位。")
    finally:
        reader.close()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("已取消；未发送运动命令", file=sys.stderr)
        raise SystemExit(130)
    except Exception as error:
        print(f"错误：{error}", file=sys.stderr)
        raise SystemExit(1)
