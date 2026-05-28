#!/bin/bash

set -eo pipefail

ROS_DISTRO="${ROS_DISTRO:-jazzy}"
CONTAINER_USERNAME="${CONTAINER_USERNAME:-robot}"
WORKSPACE_DIR="${WORKSPACE_DIR:-${HOME}/ros2_ws}"
SRC_DIR="${SRC_DIR:-${WORKSPACE_DIR}/src}"
REPO_ROOT="${REPO_ROOT:-${SRC_DIR}/robotnik/robotnik_simulation}"
EXTERNAL_REPOS_FILE="${EXTERNAL_REPOS_FILE:-${REPO_ROOT}/dependencies/repos/robotnik_simulation.${ROS_DISTRO}-devel.repos}"
CONTAINER_GROUP="${CONTAINER_GROUP:-$(id -gn "${CONTAINER_USERNAME}")}"

log() {
  echo "[devel-entrypoint] $*"
}

source_setup_file() {
  local setup_file="$1"
  if [ -f "${setup_file}" ]; then
    # ROS setup scripts are not always compatible with nounset semantics.
    set +u
    # shellcheck disable=SC1090
    source "${setup_file}"
    set -u
  fi
}

prepare_workspace_permissions() {
  mkdir -p "${WORKSPACE_DIR}" "${SRC_DIR}" "${WORKSPACE_DIR}/build" "${WORKSPACE_DIR}/install" "${WORKSPACE_DIR}/log"
  chown -R "${CONTAINER_USERNAME}:${CONTAINER_GROUP}" "${WORKSPACE_DIR}"
}

install_local_debs() {
  local deb_dir="${REPO_ROOT}/debs"
  local deb_packages=(
    ros-jazzy-robotnik-common-msgs
    ros-jazzy-robotnik-controllers-msgs
    ros-jazzy-robotnik-controllers
  )

  if ! compgen -G "${deb_dir}/*.deb" > /dev/null; then
    log "No local Robotnik debs found, skipping deb installation."
    return 0
  fi

  if ! dpkg-query -W "${deb_packages[@]}" >/dev/null 2>&1; then
    log "Installing local Robotnik debs from ${deb_dir}"
    apt-get update
    apt-get install -y "${deb_dir}"/*.deb
  else
    log "Local Robotnik debs already installed."
  fi
}

bootstrap_workspace() {
  mkdir -p "${SRC_DIR}/robotnik"
  gosu "${CONTAINER_USERNAME}:${CONTAINER_GROUP}" /usr/local/bin/bootstrap-workspace.sh "${SRC_DIR}" "${EXTERNAL_REPOS_FILE}"
}

install_rosdeps() {
  if [ ! -d "${SRC_DIR}" ]; then
    log "Workspace source directory ${SRC_DIR} not found, skipping rosdep install."
    return 0
  fi

  log "Refreshing apt package indexes before rosdep install"
  apt-get update
  log "Installing remaining rosdep dependencies from ${SRC_DIR}"
  rosdep install \
    --from-paths "${SRC_DIR}" \
    --ignore-src \
    -r \
    -y \
    --rosdistro "${ROS_DISTRO}" \
    --skip-keys "warehouse_ros_mongo"
}

main() {
  source_setup_file "/opt/ros/${ROS_DISTRO}/setup.bash"
  source_setup_file "${WORKSPACE_DIR}/install/setup.bash"

  prepare_workspace_permissions
  bootstrap_workspace
  install_local_debs
  install_rosdeps

  if [ "$#" -eq 0 ]; then
    exec gosu "${CONTAINER_USERNAME}:${CONTAINER_GROUP}" sleep infinity
  fi

  exec gosu "${CONTAINER_USERNAME}:${CONTAINER_GROUP}" "$@"
}

main "$@"
