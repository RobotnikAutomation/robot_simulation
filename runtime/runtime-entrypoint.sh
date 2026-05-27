#!/bin/bash

set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CONTAINER_USERNAME="${CONTAINER_USERNAME:-robot}"
WORKSPACE_DIR="${WORKSPACE_DIR:-/opt/robotnik_ws}"
RUNTIME_AUTOSTART="${RUNTIME_AUTOSTART:-true}"
ROBOTNIK_LAUNCH_PACKAGE="${ROBOTNIK_LAUNCH_PACKAGE:-robotnik_simulation_bringup}"
ROBOTNIK_LAUNCH_FILE="${ROBOTNIK_LAUNCH_FILE:-bringup_complete.launch.py}"
ROBOTNIK_LAUNCH_ARGS="${ROBOTNIK_LAUNCH_ARGS:-robot:=rbwatcher robot_model:=rbwatcher use_gui:=true use_rviz:=false}"

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

main() {
  source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"
  source_setup_file "${WORKSPACE_DIR}/install/setup.bash"

  if [ "$#" -gt 0 ]; then
    exec "$@"
  fi

  if [ "${RUNTIME_AUTOSTART}" = "true" ]; then
    log "Launching ${ROBOTNIK_LAUNCH_PACKAGE} ${ROBOTNIK_LAUNCH_FILE}"
    read -r -a launch_args <<< "${ROBOTNIK_LAUNCH_ARGS}"
    exec ros2 launch "${ROBOTNIK_LAUNCH_PACKAGE}" "${ROBOTNIK_LAUNCH_FILE}" "${launch_args[@]}"
  fi

  log "Runtime autostart disabled, keeping container alive."
  exec sleep infinity
}

main "$@"
