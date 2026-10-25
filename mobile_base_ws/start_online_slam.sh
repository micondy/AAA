#!/usr/bin/env bash
# 在底盘程序已经运行的前提下，启动 D435i + RTAB-Map 实时建图。
set -eo pipefail

base_workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
research_workspace="${ROBOT_RESEARCH_WS:-$(cd -- "${base_workspace}/../robot_research_ws" && pwd)}"
database_path="${SLAM_DATABASE_PATH:-${HOME}/.ros/mobile_base_online.db}"
camera_serial="${D435I_SERIAL:-}"

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
if [[ ! -r "${base_workspace}/install/local_setup.bash" ]]; then
  echo "错误：底盘工作空间尚未构建：${base_workspace}" >&2
  exit 1
fi
if [[ ! -r "${research_workspace}/install/local_setup.bash" ]]; then
  echo "错误：科研工作空间尚未构建：${research_workspace}" >&2
  exit 1
fi
source "${base_workspace}/install/local_setup.bash"
source "${research_workspace}/install/local_setup.bash"

if ! timeout 6 ros2 topic echo --once /swerve_controller/odom >/dev/null 2>&1; then
  echo "错误：6 秒内未收到 /swerve_controller/odom。请先在另一终端运行 ./start_robot.sh。" >&2
  exit 1
fi

echo "启动在线 SLAM（实时 D435i + 底盘里程计）"
echo "地图话题：/rtabmap/map"
echo "数据库：${database_path}（默认续建，不删除）"
echo "注意：相机安装外参仍须按实车标定。"

exec ros2 launch rtabmap_bringup mobile_base_online.launch.py \
  serial_no:="${camera_serial}" \
  database_path:="${database_path}" \
  fresh_map:=false \
  use_rviz:=false \
  camera_x:="${CAMERA_X:-0.10}" \
  camera_y:="${CAMERA_Y:-0.0}" \
  camera_z:="${CAMERA_Z:-1.12}" \
  camera_roll:="${CAMERA_ROLL:-0.0}" \
  camera_pitch:="${CAMERA_PITCH:-0.20944}" \
  camera_yaw:="${CAMERA_YAW:-0.0}"
