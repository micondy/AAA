#!/usr/bin/env bash
set -euo pipefail

# 已确认：can0 对应中菱行走总线，can1 对应达妙转向总线。
# DM-J4340P-2EC 24V 第一阶段调试采用 1 Mbps 经典 CAN，暂不启用 CAN-FD。
sudo ip link set can0 down 2>/dev/null || true
sudo ip link set can1 down 2>/dev/null || true
sudo ip link set can0 type can bitrate 500000 restart-ms 100
sudo ip link set can1 type can bitrate 1000000 restart-ms 100
sudo ip link set can0 up
sudo ip link set can1 up
ip -details -statistics link show can0
ip -details -statistics link show can1
