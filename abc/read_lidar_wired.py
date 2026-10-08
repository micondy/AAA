"""Read a FishBot/YDS2MD-1 lidar from a Linux serial port without ROS.

Run: python3 read_lidar_wired.py --port /dev/ttyUSB0
"""

import argparse
import csv
import os
import select
import termios
import time
import tty
from pathlib import Path

from read_lidar_wifi import PacketStream


def main():
    parser = argparse.ArgumentParser(description="Read lidar points directly from a USB serial port")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="Serial port connected to the lidar")
    parser.add_argument("--csv", type=Path, help="Save decoded points to CSV")
    parser.add_argument("--raw", type=Path, help="Save all received bytes for protocol diagnosis")
    parser.add_argument("--seconds", type=float, default=0,
                        help="Stop after this many seconds; 0 means run until Ctrl+C")
    args = parser.parse_args()

    fd = os.open(args.port, os.O_RDONLY | os.O_NOCTTY | os.O_NONBLOCK)
    old_settings = termios.tcgetattr(fd)
    csv_file = None
    raw_file = None
    try:
        tty.setraw(fd, termios.TCSANOW)
        settings = termios.tcgetattr(fd)
        settings[4] = termios.B115200
        settings[5] = termios.B115200
        termios.tcsetattr(fd, termios.TCSANOW, settings)

        if args.csv:
            csv_file = args.csv.open("a", newline="", encoding="utf-8")
            writer = csv.writer(csv_file)
            if csv_file.tell() == 0:
                writer.writerow(("timestamp", "angle_deg", "distance_m"))
        if args.raw:
            raw_file = args.raw.open("ab")

        decoder = PacketStream()
        bytes_seen = 0
        good_packets = 0
        started = last_report = time.monotonic()
        print(f"正在读取 {args.port}，115200 8N1；按 Ctrl+C 停止。", flush=True)
        while not args.seconds or time.monotonic() - started < args.seconds:
            ready, _, _ = select.select([fd], [], [], 1.0)
            if ready:
                try:
                    data = os.read(fd, 8192)
                except BlockingIOError:
                    data = b""
                if data:
                    bytes_seen += len(data)
                    if raw_file:
                        raw_file.write(data)
                    for points in decoder.feed(data):
                        good_packets += 1
                        if csv_file:
                            timestamp = time.time()
                            writer.writerows((timestamp, angle, distance)
                                            for angle, distance in points if distance > 0)
                        if good_packets % 10 == 0:
                            valid = [(a, d) for a, d in points if d > 0]
                            preview = "  ".join(
                                f"{a:.1f}°={d:.2f}m" for a, d in valid[:5]
                            )
                            print(f"第 {good_packets} 包，{len(valid)} 个有效点：{preview}", flush=True)
            if time.monotonic() - last_report >= 5:
                print(f"状态：收到 {bytes_seen} 字节，解码 {good_packets} 包，"
                      f"校验失败 {decoder.bad_packets} 包", flush=True)
                if csv_file:
                    csv_file.flush()
                if raw_file:
                    raw_file.flush()
                last_report = time.monotonic()
        print(f"结束：收到 {bytes_seen} 字节，解码 {good_packets} 包，"
              f"校验失败 {decoder.bad_packets} 包。")
    except KeyboardInterrupt:
        print("\n已停止。")
    finally:
        if csv_file:
            csv_file.close()
        if raw_file:
            raw_file.close()
        termios.tcsetattr(fd, termios.TCSANOW, old_settings)
        os.close(fd)


if __name__ == "__main__":
    main()
