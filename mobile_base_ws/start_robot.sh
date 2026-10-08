#!/usr/bin/env bash
# 底盘上电后的统一入口：配置 CAN、验证全部驱动器，再启动 ROS 2 与网页。
# ROS 2 的 setup.bash 会主动检查若干尚未定义的环境变量，因此 source 完成前
# 不能启用 nounset（set -u）。
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
runtime_config="${MOBILE_BASE_CHASSIS_CONFIG:-${HOME}/.config/mobile_base/chassis.yaml}"

source /opt/ros/jazzy/setup.bash
if [[ ! -f "${workspace_dir}/install/setup.bash" ]]; then
  echo "错误：找不到 install/setup.bash，请先在工作空间运行 colcon build --symlink-install。" >&2
  exit 1
fi
source "${workspace_dir}/install/setup.bash"
set -u

if [[ ! -f "${runtime_config}" ]]; then
  echo "错误：找不到运行配置 ${runtime_config}" >&2
  exit 1
fi

echo "[1/4] 配置 can0=500 kbit/s、can1=1 Mbit/s"
ros2 run mobile_base_bringup setup_can.sh

echo "[2/4] 验证四台达妙均可通信且为 pos_vel 模式"
ros2 run mobile_base_bringup configure_damiao_mode.py \
  --config "${runtime_config}" --interface can1 --timeout 1.0 \
  read --require-mode pos_vel

echo "[3/4] 验证中菱节点 5、6 通信和 Profile Velocity 模式"
ros2 run mobile_base_bringup test_zlac8015d.py --interface can0 --node 5 --timeout 0.8 status
ros2 run mobile_base_bringup test_zlac8015d.py --interface can0 --node 6 --timeout 0.8 status

echo "[4/4] 检查全部通过，启动底盘和局域网页"
exec ros2 launch mobile_base_bringup robot.launch.py chassis_config:="${runtime_config}"
