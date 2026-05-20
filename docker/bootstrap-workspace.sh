#!/bin/bash

set -euo pipefail

SRC_DIR="${1:?source workspace path is required}"
MANIFEST_FILE="${2:?external repos manifest path is required}"

required_repos=(
  "robotnik/robotnik_description"
  "robotnik/robotnik_sensors"
  "robotnik/robotnik_common"
  "robotnik/robotnik_interfaces"
  "robotnik/robotnik_moveit_configs"
  "robotnik/teleop_panel"
)

missing_repo=false

for repo_path in "${required_repos[@]}"; do
  if [ ! -d "${SRC_DIR}/${repo_path}" ]; then
    missing_repo=true
    break
  fi
done

if [ "${missing_repo}" = true ]; then
  echo "[bootstrap-workspace] Importing missing external repositories into ${SRC_DIR}"
  vcs import --skip-existing "${SRC_DIR}" < "${MANIFEST_FILE}"
else
  echo "[bootstrap-workspace] External repositories already present, skipping import."
fi
