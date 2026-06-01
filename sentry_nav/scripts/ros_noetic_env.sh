#!/usr/bin/env bash
set -euo pipefail

# Start from a ROS-safe shell even if the user opened the terminal in conda base.
if [[ -n "${CONDA_SHLVL:-}" && "${CONDA_SHLVL}" != "0" ]]; then
  if command -v conda >/dev/null 2>&1; then
    eval "$(conda shell.bash hook)"
    while [[ -n "${CONDA_SHLVL:-}" && "${CONDA_SHLVL}" != "0" ]]; do
      conda deactivate
    done
  fi
fi

unset PYTHONHOME
export PYTHONNOUSERSITE=1

# Remove miniconda prefixes from PATH for this shell.
if [[ -n "${PATH:-}" ]]; then
  CLEAN_PATH=""
  OLD_IFS="$IFS"
  IFS=':'
  for entry in $PATH; do
    if [[ "$entry" == "/home/zhl/miniconda3/bin" || "$entry" == "/home/zhl/miniconda3/condabin" ]]; then
      continue
    fi
    if [[ -z "$CLEAN_PATH" ]]; then
      CLEAN_PATH="$entry"
    else
      CLEAN_PATH="${CLEAN_PATH}:$entry"
    fi
  done
  IFS="$OLD_IFS"
  export PATH="$CLEAN_PATH"
fi

source /opt/ros/noetic/setup.bash

if [[ -f "/home/zhl/Livox_ws/devel/setup.bash" ]]; then
  source /home/zhl/Livox_ws/devel/setup.bash
fi

if [[ -f "/home/zhl/robot3/nav_ws/devel/setup.bash" ]]; then
  source /home/zhl/robot3/nav_ws/devel/setup.bash
fi

echo "ROS_PYTHON=$(command -v python3)"
echo "ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH:-}"
