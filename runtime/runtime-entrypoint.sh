#!/bin/bash

set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CONTAINER_USERNAME="${CONTAINER_USERNAME:-robot}"
WORKSPACE_DIR="${WORKSPACE_DIR:-/opt/robotnik_ws}"
RUNTIME_AUTOSTART="${RUNTIME_AUTOSTART:-true}"
ROBOTNIK_LAUNCH_PACKAGE="${ROBOTNIK_LAUNCH_PACKAGE:-robotnik_simulation_bringup}"
ROBOTNIK_LAUNCH_FILE="${ROBOTNIK_LAUNCH_FILE:-bringup_complete.launch.py}"
ROBOTNIK_LAUNCH_ARGS="${ROBOTNIK_LAUNCH_ARGS:-}"

log() {
  echo "[runtime-entrypoint] $*"
}

source_setup_file() {
  local setup_file="$1"
  if [ -f "${setup_file}" ]; then
    set +u
    # shellcheck disable=SC1090
    source "${setup_file}"
    set -u
  fi
}

append_launch_arg() {
  local arg_name="$1"
  local arg_value="$2"

  if [ -n "${arg_value}" ]; then
    launch_args+=("${arg_name}:=${arg_value}")
  fi
}

resolve_world_path() {
  if [ -n "${WORLD_PATH:-}" ]; then
    printf '%s\n' "${WORLD_PATH}"
    return 0
  fi

  if [ -n "${WORLD:-}" ]; then
    local pkg_prefix
    pkg_prefix="$(ros2 pkg prefix robotnik_gazebo_ignition)"
    printf '%s/share/robotnik_gazebo_ignition/worlds/%s.world\n' "${pkg_prefix}" "${WORLD}"
    return 0
  fi

  printf '\n'
}

main() {
  source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"
  source_setup_file "${WORKSPACE_DIR}/install/setup.bash"

  if [ "$#" -gt 0 ]; then
    exec "$@"
  fi

  if [ "${RUNTIME_AUTOSTART}" = "true" ]; then
    log "Launching ${ROBOTNIK_LAUNCH_PACKAGE} ${ROBOTNIK_LAUNCH_FILE}"
    if [ -n "${ROBOTNIK_LAUNCH_ARGS}" ]; then
      read -r -a launch_args <<< "${ROBOTNIK_LAUNCH_ARGS}"
      exec ros2 launch "${ROBOTNIK_LAUNCH_PACKAGE}" "${ROBOTNIK_LAUNCH_FILE}" "${launch_args[@]}"
    fi

    launch_args=()
    append_launch_arg "robot_id" "${ROBOT_ID:-}"
    append_launch_arg "robot" "${ROBOT:-}"
    append_launch_arg "robot_model" "${ROBOT_MODEL:-}"
    append_launch_arg "robot_xacro_path" "${ROBOT_XACRO_PATH:-}"
    append_launch_arg "use_gui" "${USE_GUI:-}"
    append_launch_arg "low_performance_simulation" "${LOW_PERFORMANCE_SIMULATION:-}"
    append_launch_arg "use_rviz" "${USE_RVIZ:-}"
    append_launch_arg "frame_prefix" "${FRAME_PREFIX:-}"
    append_launch_arg "run_moveit" "${RUN_MOVEIT:-}"
    append_launch_arg "arm_type" "${ARM_TYPE:-}"
    append_launch_arg "world_path" "$(resolve_world_path)"

    exec ros2 launch "${ROBOTNIK_LAUNCH_PACKAGE}" "${ROBOTNIK_LAUNCH_FILE}" "${launch_args[@]}"
  fi

  log "Runtime autostart disabled, keeping container alive."
  exec sleep infinity
}

main "$@"
