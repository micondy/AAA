#!/usr/bin/env bash
# 带 ROS 环境启动草菇采集台（用于 D435i 快照 / rosbag 录制）
#
#   ./start_with_stack.sh                                          # 只载入 /opt/ros/*
#   ./start_with_stack.sh /media/scc/Windows_D/AAA/robot_research_ws
#   GM_WORKSPACE=/path/to/ws ./start_with_stack.sh --open
#
# 工作区优先用 tools/source_robot_stack.bash（多层 underlay），
# 没有就用 install/setup.bash。
set -euo pipefail
cd "$(dirname "$0")"

WS="${GM_WORKSPACE:-}"
if [[ $# -gt 0 && -d "${1:-}" ]]; then
  WS="$1"
  shift
fi

if [[ -z "${ROS_DISTRO:-}" ]]; then
  for candidate in /opt/ros/jazzy/setup.bash /opt/ros/*/setup.bash; do
    if [[ -r "$candidate" ]]; then
      # ROS 的 setup.bash 不兼容 set -u，source 期间先关掉
      set +u
      # shellcheck disable=SC1090
      source "$candidate"
      set -u
      break
    fi
  done
fi

if [[ -n "$WS" ]]; then
  WS="$(cd "$WS" && pwd)"
  set +u
  if [[ -r "$WS/tools/source_robot_stack.bash" ]]; then
    # shellcheck disable=SC1090
    source "$WS/tools/source_robot_stack.bash"
  elif [[ -r "$WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1090
    source "$WS/install/setup.bash"
    echo "已 source $WS/install/setup.bash"
  else
    set -u
    echo "找不到 $WS 的 setup 文件（既无 tools/source_robot_stack.bash 也无 install/setup.bash）" >&2
    exit 1
  fi
  set -u
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "提示：PATH 里没有 ros2，将只启用拍照与测量（D435i/录包按钮会置灰）。" >&2
fi

exec python3 app.py "$@"
