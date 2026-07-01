#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ] || [ "$#" -gt 6 ]; then
  echo "Usage: $0 <x> <y> <yaw_deg> [slow|fast] [max_vel] [max_wz]"
  echo "Example: $0 1.0 0.5 90.0 slow 0.4 0.8"
  exit 1
fi

profile_name="${4:-slow}"
case "$profile_name" in
  slow|0)
    pid_profile=0
    ;;
  fast|1)
    pid_profile=1
    ;;
  *)
    echo "Invalid pid profile: $profile_name"
    echo "Use slow, fast, 0, or 1."
    exit 1
    ;;
esac

max_vel="${5:-0.0}"
max_wz="${6:-0.0}"
goal="{x: $1, y: $2, yaw_deg: $3, pid_profile: $pid_profile, max_vel: $max_vel, max_wz: $max_wz}"
echo "Sending goal to /move_to_pose as action_of_motion_interfaces/action/MoveToPose: $goal"
ros2 action send_goal /move_to_pose action_of_motion_interfaces/action/MoveToPose "$goal" \
  --feedback
