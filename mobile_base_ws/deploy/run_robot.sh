#!/usr/bin/env bash
set -eo pipefail

# 脚本位置决定工作空间位置，默认工程安装在 /opt/mobile_base_ws。
script_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
workspace_dir="$(cd -- "${script_dir}/.." && pwd)"

# 优先使用当前 ROS 环境；未加载环境时选择本机已安装的发行版。
ros_distro="${ROS_DISTRO:-}"
if [[ -z "${ros_distro}" ]]; then
  if [[ -r /opt/ros/humble/setup.bash ]]; then
    ros_distro=humble
  elif [[ -r /opt/ros/jazzy/setup.bash ]]; then
    ros_distro=jazzy
  fi
fi
if [[ ! -r "/opt/ros/${ros_distro}/setup.bash" ]]; then
  echo "错误：找不到 ROS 环境，请先 source /opt/ros/<发行版>/setup.bash。" >&2
  exit 1
fi
source "/opt/ros/${ros_distro}/setup.bash"
source "${workspace_dir}/install/setup.bash"
set -u

export MOBILE_BASE_CONFIG_DIR="${MOBILE_BASE_CONFIG_DIR:-${workspace_dir}/config}"
config_file="${MOBILE_BASE_CONFIG_DIR}/chassis.yaml"
if [[ ! -f "${config_file}" ]]; then
  echo "找不到机器人端底盘配置：${config_file}" >&2
  exit 2
fi

exec ros2 launch mobile_base_bringup robot.launch.py start_web_console:=true
