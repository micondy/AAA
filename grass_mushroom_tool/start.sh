#!/usr/bin/env bash
# 草菇采集台启动脚本（独立工具，不改动任何现有工程）
#
#   ./start.sh                 普通启动（仅拍照 + 测量记录）
#   ./start.sh --open          启动并自动打开浏览器
#   GM_SETUP=/opt/ros/jazzy/setup.bash ./start.sh   先载入指定的 ROS 环境
#
set -euo pipefail
cd "$(dirname "$0")"

# 已经在 ROS 环境里就不再重复 source
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

# 可选：把自己的工作区 setup 文件塞进来（例如要用自定义相机前缀）
if [[ -n "${GM_SETUP:-}" && -r "${GM_SETUP}" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "${GM_SETUP}"
  set -u
fi

exec python3 app.py "$@"
