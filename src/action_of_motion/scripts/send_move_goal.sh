#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 3 ] || [ "$#" -gt 4 ]; then
  echo "Usage: $0 <x> <y> <yaw_deg> [slow|fast]"
  echo "Example: $0 1.0 0.5 90.0 slow"
  exit 1
fi

profile_name="${4:-slow}"
case "$profile_name" in
  slow)
    pid_profile=0
    ;;
  fast)
    pid_profile=1
    ;;
  *)
    echo "Invalid pid profile: $profile_name"
    echo "Use slow or fast."
    exit 1
    ;;
esac

ros2 action send_goal /move_to_pose action_of_motion_interfaces/action/MoveToPose \
  "{x: $1, y: $2, yaw_deg: $3, pid_profile: $pid_profile}" \
  --feedback
