#!/usr/bin/env bash
# Source the mobile-manipulation stack in deterministic underlay/overlay order.
# Usage: source /media/scc/Windows_D/AAA/robot_research_ws/tools/source_robot_stack.bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "This file must be sourced, not executed." >&2
  exit 2
fi

# Remove a previously sourced personal OpenArm workspace from this shell. This is necessary because
# ROS setup files prepend paths but do not remove stale underlays already present in the environment.
_robot_stack_excluded_prefix="/home/scc/ros2_ws"
_robot_stack_path_vars=(
  AMENT_PREFIX_PATH
  CMAKE_PREFIX_PATH
  COLCON_PREFIX_PATH
  LD_LIBRARY_PATH
  PATH
  PKG_CONFIG_PATH
  PYTHONPATH
)

for _robot_stack_var_name in "${_robot_stack_path_vars[@]}"; do
  declare -n _robot_stack_var_ref="${_robot_stack_var_name}"
  _robot_stack_cleaned=""
  IFS=: read -r -a _robot_stack_entries <<< "${_robot_stack_var_ref:-}"
  for _robot_stack_entry in "${_robot_stack_entries[@]}"; do
    if [[ -n "${_robot_stack_entry}" && "${_robot_stack_entry}" != "${_robot_stack_excluded_prefix}"* ]]; then
      _robot_stack_cleaned+="${_robot_stack_cleaned:+:}${_robot_stack_entry}"
    fi
  done
  _robot_stack_var_ref="${_robot_stack_cleaned}"
  export "${_robot_stack_var_name}"
  unset -n _robot_stack_var_ref
done

_robot_stack_setups=(
  "/opt/ros/jazzy/setup.bash"
  "/media/scc/Windows_D/AAA/mobile_base_ws/install/local_setup.bash"
  "/media/scc/Windows_D/AAA/robot_research_ws/install/local_setup.bash"
)

for _robot_stack_setup in "${_robot_stack_setups[@]}"; do
  if [[ ! -r "${_robot_stack_setup}" ]]; then
    echo "Missing required setup: ${_robot_stack_setup}" >&2
    unset _robot_stack_setup _robot_stack_setups
    return 1
  fi
  # shellcheck disable=SC1090
  source "${_robot_stack_setup}"
done

unset _robot_stack_cleaned _robot_stack_entries _robot_stack_entry
unset _robot_stack_excluded_prefix _robot_stack_path_vars _robot_stack_var_name
unset _robot_stack_setup _robot_stack_setups
echo "Robot stack ready: ROS 2 -> mobile_base -> robot_research (pinned official OpenArm stack)"
