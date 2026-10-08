"""Read FishBot Wi-Fi adapter data on Windows without ROS or extra packages.

The connected YDS2MD-1 decoded successfully with the YDLIDAR S2/X2 protocol
over USB serial; Wi-Fi transport has not been verified on hardware.
Run: python read_lidar_wifi.py
"""

import argparse
import csv
import math
import socket
import struct
import time
from pathlib import Path


HEADER = b"\xaa\x55"
MAX_SAMPLES = 80


def decode_packet(packet):
    """Return (angle_deg, distance_m) points, or None for a bad packet."""
    if len(packet) < 12 or packet[:2] != HEADER:
        return None
    ct, count, first, last, checksum = struct.unpack_from("<BBHHH", packet, 2)
    if not 1 <= count <= MAX_SAMPLES or len(packet) != 10 + 2 * count:
        return None

    sample_values = struct.unpack_from("<" + "H" * count, packet, 10)
    calculated = 0x55AA ^ first ^ last ^ (count << 8 | ct)
    for value in sample_values:
        calculated ^= value
    if calculated != checksum:
        return None

    start_angle = (first >> 1) / 64.0
    end_angle = (last >> 1) / 64.0
    if start_angle >= 360.0 or end_angle >= 360.0:
        return None
    angle_span = (end_angle - start_angle) % 360.0
    points = []
    for index, value in enumerate(sample_values):
        distance_mm = value / 4.0
        angle = start_angle + angle_span * index / max(count - 1, 1)
        if distance_mm:
            correction = math.degrees(math.atan(
                21.8 * (155.3 - distance_mm) / (155.3 * distance_mm)
            ))
            angle += correction
        points.append((angle % 360.0, distance_mm / 1000.0))
    return points


class PacketStream:
    def __init__(self):
        self.buffer = bytearray()
        self.bad_packets = 0

    def feed(self, data):
        for _, points in self.feed_packets(data):
            yield points

    def feed_packets(self, data):
        """Yield (packet_type, points); packet_type bit 0 starts a scan."""
        self.buffer.extend(data)
        while True:
            start = self.buffer.find(HEADER)
            if start < 0:
                self.buffer[:] = self.buffer[-1:] if self.buffer[-1:] == b"\xaa" else b""
                return
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 10:
                return
            count = self.buffer[3]
            if not 1 <= count <= MAX_SAMPLES:
                self.bad_packets += 1
                del self.buffer[0]
                continue
            length = 10 + 2 * count
            if len(self.buffer) < length:
                return
            packet_type = self.buffer[2]
            points = decode_packet(self.buffer[:length])
            if points is None:
                self.bad_packets += 1
                del self.buffer[0]
                continue
            del self.buffer[:length]
            yield packet_type, points


def main():
    parser = argparse.ArgumentParser(description="Receive YDS2MD-1 data via FishBot Wi-Fi adapter")
    parser.add_argument("--host", default="0.0.0.0", help="PC address to listen on")
    parser.add_argument("--port", type=int, default=8889, help="TCP port set on adapter")
    parser.add_argument("--csv", type=Path, help="Save all decoded points to a CSV file")
    parser.add_argument("--raw", type=Path, help="Save received bytes if the protocol cannot be decoded")
    args = parser.parse_args()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((args.host, args.port))
        server.listen(1)
        print(f"正在监听 TCP {args.host}:{args.port}；转接板的 server_ip 应填这台电脑的局域网 IP。", flush=True)
        try:
            while True:
                connection, address = server.accept()
                print(f"转接板已连接：{address[0]}:{address[1]}", flush=True)
                decoder = PacketStream()
                bytes_seen = 0
                good_packets = 0
                last_report = time.monotonic()
                csv_file = None
                raw_file = None
                try:
                    if args.csv:
                        csv_file = args.csv.open("a", newline="", encoding="utf-8")
                        writer = csv.writer(csv_file)
                        if csv_file.tell() == 0:
                            writer.writerow(("timestamp", "angle_deg", "distance_m"))
                    if args.raw:
                        raw_file = args.raw.open("ab")
                    with connection:
                        connection.settimeout(1.0)
                        while True:
                            try:
                                data = connection.recv(8192)
                            except socket.timeout:
                                data = None
                            if data == b"":
                                break
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
                except (ConnectionError, OSError) as exc:
                    print(f"连接中断：{exc}", flush=True)
                finally:
                    if csv_file:
                        csv_file.close()
                    if raw_file:
                        raw_file.close()
                print("连接已断开，继续等待转接板。", flush=True)
        except KeyboardInterrupt:
            print("\n已停止。")


if __name__ == "__main__":
    main()
