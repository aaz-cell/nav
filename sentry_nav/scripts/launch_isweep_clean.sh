#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/../../../.." && pwd)"
source "${SCRIPT_DIR}/ros_noetic_env.sh"

cd "${WORKSPACE_DIR}"

exec roslaunch sentry_nav navigation_isweep_switch.launch \
  run_legacy_move_base:=false \
  enable_isweep_cmd_output:=false \
  cmd_vel_topic:=/isweep_cmd_vel \
  "$@"
