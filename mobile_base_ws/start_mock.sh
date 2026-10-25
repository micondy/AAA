#!/usr/bin/env bash
# 无电机连接的底盘与网页演示；使用独立 ROS domain 和配置目录。
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
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
if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  echo "错误：找不到 install/setup.bash，请先在工作空间运行 colcon build --symlink-install。" >&2
  exit 1
fi
source "${workspace_dir}/install/setup.bash"
set -u


export ROS_DOMAIN_ID="${MOBILE_BASE_MOCK_DOMAIN_ID:-87}"
export MOBILE_BASE_CONFIG_DIR="${MOBILE_BASE_MOCK_CONFIG_DIR:-${HOME}/.config/mobile_base_mock}"
exec ros2 launch mobile_base_bringup mock.launch.py use_rviz:=false start_web_console:=true "$@"
