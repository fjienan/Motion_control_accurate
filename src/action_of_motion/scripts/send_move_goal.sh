#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -ne 3 ]; then
  echo "Usage: $0 <x> <y> <yaw_deg>"
  echo "Example: $0 1.0 0.5 90.0"
  exit 1
fi

ros2 action send_goal /move_to_pose action_of_motion_interfaces/action/MoveToPose \
  "{x: $1, y: $2, yaw_deg: $3}" \
  --feedback
