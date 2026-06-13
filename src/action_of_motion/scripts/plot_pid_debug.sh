#!/usr/bin/env bash
set -euo pipefail

PREFIX="${1:-/move_to_pose/debug}"

rqt_plot \
  "${PREFIX}/distance_error/data" \
  "${PREFIX}/yaw_error_deg/data" \
  "${PREFIX}/cmd_linear_speed/data" \
  "${PREFIX}/cmd_vx/data" \
  "${PREFIX}/cmd_vy/data" \
  "${PREFIX}/cmd_wz/data"
