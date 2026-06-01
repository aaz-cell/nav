#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "${SCRIPT_DIR}/ros_noetic_env.sh"

cd /home/zhl/robot3/nav_ws

exec roslaunch sentry_nav navigation_isweep_switch.launch \
  run_legacy_move_base:=false \
  enable_isweep_cmd_output:=false \
  cmd_vel_topic:=/isweep_cmd_vel \
  "$@"
