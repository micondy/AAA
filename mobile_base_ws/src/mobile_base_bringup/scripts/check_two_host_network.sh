#!/usr/bin/env bash
set -uo pipefail

role=""
peer=""

usage() {
  cat <<'EOF'
用法：
  check_two_host_network.sh --role robot|operator [--peer 对端IP]

说明：
  robot   检查底盘电脑的 ROS 环境、CAN 接口和本机节点。
  operator 检查操作端能否发现机器人端关键节点与话题。
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --role)
      role="${2:-}"
      shift 2
      ;;
    --peer)
      peer="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "未知参数：$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ "${role}" != "robot" && "${role}" != "operator" ]]; then
  echo "--role 必须是 robot 或 operator" >&2
  exit 2
fi

failures=0

ok() { echo "[通过] $*"; }
warn() { echo "[警告] $*"; }
fail() { echo "[失败] $*"; failures=$((failures + 1)); }

if ! command -v ros2 >/dev/null 2>&1; then
  fail "找不到 ros2，请先 source /opt/ros/jazzy/setup.bash 和工作空间。"
else
  ok "已找到 ros2：$(command -v ros2)"
fi

if [[ "${ROS_DISTRO:-}" == "jazzy" ]]; then
  ok "ROS_DISTRO=jazzy"
else
  fail "ROS_DISTRO=${ROS_DISTRO:-未设置}，两端都必须使用 Jazzy。"
fi

if [[ "${ROS_DOMAIN_ID:-}" =~ ^[0-9]+$ ]]; then
  ok "ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
else
  fail "ROS_DOMAIN_ID 未设置为整数；两端必须设置成相同值。"
fi

if [[ "${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}" == "rmw_fastrtps_cpp" ]]; then
  ok "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
else
  warn "当前 RMW=${RMW_IMPLEMENTATION}；必须确认另一台主机完全相同。"
fi

case "${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}" in
  SUBNET)
    ok "DDS 自动发现范围为 SUBNET"
    ;;
  LOCALHOST|OFF)
    if [[ -z "${ROS_STATIC_PEERS:-}" ]]; then
      fail "发现范围为 ${ROS_AUTOMATIC_DISCOVERY_RANGE}，且没有 ROS_STATIC_PEERS。"
    else
      warn "使用静态发现对端：${ROS_STATIC_PEERS}"
    fi
    ;;
  *)
    warn "未知发现范围：${ROS_AUTOMATIC_DISCOVERY_RANGE}"
    ;;
esac

if [[ "${ROS_LOCALHOST_ONLY:-0}" == "1" ]]; then
  fail "ROS_LOCALHOST_ONLY=1 会禁止跨主机通信。"
fi

if [[ -n "${peer}" ]]; then
  if ping -c 1 -W 2 "${peer}" >/dev/null 2>&1; then
    ok "可以 ping 通对端 ${peer}"
  else
    fail "无法 ping 通对端 ${peer}，请检查 IP、网段和 AP 客户端隔离。"
  fi
fi

if [[ "${role}" == "robot" ]]; then
  for interface in can0 can1; do
    if ip link show "${interface}" >/dev/null 2>&1; then
      interface_state="$(ip -brief link show "${interface}" | awk '{print $2}')"
      if [[ "${interface_state}" == "UP" ]]; then
        ok "${interface} 已存在并处于 UP 状态"
      else
        fail "${interface} 存在但状态为 ${interface_state}"
      fi
    else
      fail "找不到 ${interface}"
    fi
  done
fi

if command -v ros2 >/dev/null 2>&1; then
  graph="$(timeout 8 ros2 node list 2>/dev/null || true)"
  if [[ -n "${graph}" ]]; then
    ok "ROS 2 图中发现节点"
  else
    fail "8 秒内没有发现任何 ROS 2 节点；先确认机器人端已经启动。"
  fi

  if [[ "${role}" == "operator" ]]; then
    for node in /controller_manager /robot_state_publisher; do
      if grep -Fxq "${node}" <<<"${graph}"; then
        ok "发现机器人端节点 ${node}"
      else
        fail "未发现机器人端节点 ${node}"
      fi
    done

    topics="$(timeout 8 ros2 topic list 2>/dev/null || true)"
    for topic in /joint_states /robot_description /swerve_controller/odom; do
      if grep -Fxq "${topic}" <<<"${topics}"; then
        ok "发现话题 ${topic}"
      else
        fail "未发现话题 ${topic}"
      fi
    done
  fi
fi

if (( failures > 0 )); then
  echo "检查完成：${failures} 项失败。修改 DDS 环境变量后请执行 ros2 daemon stop 再重试。" >&2
  exit 1
fi

echo "检查完成：未发现阻断项。"
