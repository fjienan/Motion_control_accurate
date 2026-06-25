import time

import rclpy
from action_of_motion_interfaces.action import MoveToPose
from rclpy.action import ActionClient


PID_PROFILE_NAMES = {
    'slow': MoveToPose.Goal.PID_PROFILE_SLOW,
    'fast': MoveToPose.Goal.PID_PROFILE_FAST,
}


class MoveToPoseClient:
    def __init__(self, node, action_name='/move_to_pose'):
        self.node = node
        self._client = ActionClient(node, MoveToPose, action_name)

    def wait_for_server(self, timeout_sec=None):
        return self._client.wait_for_server(timeout_sec=timeout_sec)

    def send_goal(
        self,
        x,
        y,
        yaw_deg,
        pid_profile,
        feedback_callback=None,
        timeout_sec=None,
    ):
        goal_msg = MoveToPose.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.yaw_deg = float(yaw_deg)
        goal_msg.pid_profile = self._normalize_pid_profile(pid_profile)

        goal_future = self._client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_callback,
        )
        self._spin_until_complete(goal_future, timeout_sec)
        goal_handle = goal_future.result()
        if goal_handle is None:
            raise RuntimeError('MoveToPose goal request did not complete')
        if not goal_handle.accepted:
            raise RuntimeError('MoveToPose goal was rejected')

        result_future = goal_handle.get_result_async()
        self._spin_until_complete(result_future, timeout_sec)
        result_response = result_future.result()
        if result_response is None:
            raise RuntimeError('MoveToPose result request did not complete')
        return result_response.result

    def send_goal_async(self, x, y, yaw_deg, pid_profile,
                        feedback_callback=None):
        goal_msg = MoveToPose.Goal()
        goal_msg.x = float(x)
        goal_msg.y = float(y)
        goal_msg.yaw_deg = float(yaw_deg)
        goal_msg.pid_profile = self._normalize_pid_profile(pid_profile)
        return self._client.send_goal_async(
            goal_msg,
            feedback_callback=feedback_callback,
        )

    def _spin_until_complete(self, future, timeout_sec):
        if timeout_sec is None:
            rclpy.spin_until_future_complete(self.node, future)
            return

        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done():
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                raise TimeoutError('Timed out waiting for MoveToPose action')
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=min(remaining, 0.1),
            )

    def _normalize_pid_profile(self, pid_profile):
        if isinstance(pid_profile, str):
            normalized = pid_profile.strip().lower()
            if normalized in PID_PROFILE_NAMES:
                return PID_PROFILE_NAMES[normalized]
        elif pid_profile in (
            MoveToPose.Goal.PID_PROFILE_SLOW,
            MoveToPose.Goal.PID_PROFILE_FAST,
        ):
            return int(pid_profile)

        raise ValueError(
            'pid_profile must be 0, 1, "slow", or "fast"'
        )
