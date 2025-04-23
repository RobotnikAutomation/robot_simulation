#!/bin/bash
#
# Copyright (c) 2024, Robotnik Automation S.L.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Robotnik Automation S.L. nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL Robotnik Automation S.L. BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# Entrypoint script, loads other scripts and executes the command

set -e

function entrypoint_log() {
  if [ -z "${ROS_ENTRYPOINT_QUIET_LOGS:-}" ]; then
    echo "$@"
  fi
}

function source_ros_entrypoint_dir() {
  if find /ros_entrypoint.d/ -mindepth 1 -maxdepth 1 -type f -print -quit 2>/dev/null | read v; then
    entrypoint_log "$0: /ros_entrypoint.d/ is not empty, will attempt to execute scripts"
    find "/ros_entrypoint.d/" -follow -type f -print | sort -V | while read -r f; do
      case "$f" in
        *.sh)
          if [ -x "$f" ]; then
            entrypoint_log "$0: running $f"
            "$f"
          else
            entrypoint_log "$0: ignoring $f, not executable"
          fi
          ;;
        *) entrypoint_log "$0: ignoring $f" ;;
      esac
    done
    entrypoint_log "$0: /ros_entrypoint.d/ has been processed successfully"
  else
    entrypoint_log "$0: No files found /ros_entrypoint.d/, skipping"
  fi
}

function use_generic() {
  exec_cmd="${*}"
  if [[ "${exec_cmd}" == "__default_cmd__" ]]; then
    exec_cmd="${GEN_COMMAND}"
  fi
  return 0
}

function use_rosmon() {
  if [[ ${ROS_VERSION} == "2" ]]; then
    entrypoint_log "$0: ros mon is not supported on ros2"
    return 1
  fi
  rosmon_name="${ROBOT_ID}_rosmon_${ROS_BU_DESC}"
  exec_cmd="rosrun rosmon_core rosmon"
  exec_cmd="${exec_cmd} --flush-stdout"
  exec_cmd="${exec_cmd} --disable-ui"
  exec_cmd="${exec_cmd} --name=${rosmon_name}"
  exec_cmd="${exec_cmd} ${ROS_BU_PKG}"
  exec_cmd="${exec_cmd} ${ROS_BU_LAUNCH}"
  return 0
}

function use_ros_run() {
  if [[ ${ROS_VERSION} == "1" ]]; then
    exec_cmd="rosrun"
  fi
  if [[ ${ROS_VERSION} == "2" ]]; then
    exec_cmd="ros2 run"
  fi
  exec_cmd="${exec_cmd} ${ROS_BU_PKG}"
  exec_cmd="${exec_cmd} ${ROS_BU_LAUNCH}"
  return 0
}

function use_ros_launch() {
  if [[ ${ROS_VERSION} == "1" ]]; then
    exec_cmd="roslaunch"
  fi
  if [[ ${ROS_VERSION} == "2" ]]; then
    exec_cmd="ros2 launch"
  fi
  exec_cmd="${exec_cmd} ${ROS_BU_PKG}"
  exec_cmd="${exec_cmd} ${ROS_BU_LAUNCH}"
  return 0
}

function fake_screen_startup() {
  if ! is_graphical; then
    return 1
  fi
  # if the graphical start is selected do nothing
  if ! [[ "${GRAPHICAL_START}" == true ]]; then
    return 0
  fi
  export GUI_COMMAND="${exec_cmd}"
  exec_cmd="/usr/local/bin/vnc_launcher.sh"
}

function is_graphical() {
  if ! [[ -r "/usr/local/bin/vnc_launcher.sh" ]]; then
    entrypoint_log "$0: VNC base graphical execution is not allowed on this version"
    return 1
  fi
  GRAPHICAL_START="true"
  return 0
}

function use_graphical() {
  if ! is_graphical; then
    return 1
  fi
  exec_cmd="/usr/local/bin/vnc_launcher.sh"
  return 0
}

function select_ros_launcher() {
  local startup_func="use_generic"
  if [[ "${*}" == "__default_cmd__" ]]; then
    case "${STARTUP_TYPE}" in
      generic)
        startup_func="use_generic"
        ;;
      rosmon)
        startup_func="use_rosmon"
        ;;
      launch)
        startup_func="use_ros_launch"
        ;;
      run)
        startup_func="use_ros_run"
        ;;
      graphical)
        startup_func="use_graphical"
        ;;
      *)
        entrypoint_log "$0: not valid"
        return 1
        ;;
    esac
  else
    entrypoint_log "$0: custom command detected, ignoring STARTUP_TYPE"
  fi
  if ! eval "${startup_func} ${@}"; then
    return 1
  fi
  if [[ ${FAKE_SCREEN} == "true" ]]; then
    fake_screen_startup
    return $?
  fi
  return 0
}

function check_nodes() {
  # Check if enabled and nodes are set
  if ! [[ ${CHECK_NODES} == "true" ]]; then
    return 0
  fi
  if [[ -z ${NODES_TO_CHECK} ]]; then
    return 0
  fi

  # Wait for nodes to be available
  entrypoint_log "$0: env variable CHECK_NODES is set to true, waiting for nodes to be available: \"${NODES_TO_CHECK}\""
  entrypoint_log "$0: ---"
  i=0
  while true; do
    i=$((i + 1))
    if eval "${HEALTHCHECK_EXEC} ${NODES_TO_CHECK} > /tmp/healthcheck.log"; then
      return 0
    fi
    if [[ $(( i%5 )) -eq 0 ]]
    then
      i=0
      cat /tmp/healthcheck.log
      entrypoint_log "$0: ---"
    fi
    sleep 0.25
  done
  entrypoint_log "$0: error: at least one of follwing nodes is not available: \"${NODES_TO_CHECK}\""
  return 1
}

function _trap_handler() {
  entrypoint_log "$0: caught signal \"$1\" before starting main process, exiting"
  exit 0
}

function main() {
  # Trap SIGTERM and SIGINT to stop the container gracefully
  # it only works before the exec command
  trap '_trap_handler SIGTERM' SIGTERM
  trap '_trap_handler SIGINT' SIGINT

  # Source all scripts in /ros_entrypoint.d/
  source_ros_entrypoint_dir

  # Set up ROS environment
  if ! source env_loader.sh
  then
    entrypoint_log "$0: error loading ROS environment, check logs"
  fi

  # Fill exec_cmd
  if ! select_ros_launcher "${@}"
  then
    return 1
  fi

  # substitute the environment variables
  exec_cmd=$(eval echo "${exec_cmd}")
  entrypoint_log "$0: executing ${exec_cmd}"
  if ! check_nodes
  then
    return 1
  fi
  entrypoint_log "$0: all nodes are available, executing ${exec_cmd}"
  exec /usr/bin/tini -- ${exec_cmd}
}

main "${@}"
