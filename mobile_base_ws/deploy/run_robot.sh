#!/usr/bin/env bash
set -euo pipefail

# 脚本位置决定工作空间位置，默认工程安装在 /opt/mobile_base_ws。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"

source /opt/ros/jazzy/setup.bash
source "${workspace_dir}/install/setup.bash"

export MOBILE_BASE_CONFIG_DIR="${MOBILE_BASE_CONFIG_DIR:-/opt/mobile_base_ws/config}"
config_file="${MOBILE_BASE_CONFIG_DIR}/chassis.yaml"
if [[ ! -f "${config_file}" ]]; then
  echo "找不到机器人端底盘配置：${config_file}" >&2
  exit 2
fi

exec ros2 launch mobile_base_bringup robot.launch.py start_web_console:=true
