import math
import time
from dataclasses import dataclass

import rclpy
from action_of_motion_interfaces.action import MoveToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray


ACTION_NAME = 'move_to_pose'
PHASE_WAITING_FOR_POSE = 'waiting_for_pose'
PHASE_YAW = 'yaw'
PHASE_XY = 'xy'


@dataclass
class Pose2D:
    x: float
    y: float
    yaw_rad: float


class PidController:
    def __init__(self, kp, ki, kd, integral_limit):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = abs(integral_limit)
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.previous_error = None

    def update(self, error, dt):
        if dt <= 0.0:
            derivative = 0.0
        elif self.previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt

        self.integral += error * max(dt, 0.0)
        self.integral = clamp(
            self.integral,
            -self.integral_limit,
            self.integral_limit,
        )
        self.previous_error = error

        return (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )


def clamp(value, minimum, maximum):
    return max(minimum, min(maximum, value))


def apply_min_output(value, minimum):
    minimum = abs(minimum)
    if minimum <= 0.0 or value == 0.0:
        return value
    if abs(value) < minimum:
        return math.copysign(minimum, value)
    return value


def normalize_angle(angle_rad):
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def map_error_to_body(dx_map, dy_map, yaw_rad):
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    x_body = cos_yaw * dx_map + sin_yaw * dy_map
    y_body = -sin_yaw * dx_map + cos_yaw * dy_map
    return x_body, y_body


def limit_vector(x_value, y_value, max_norm):
    max_norm = abs(max_norm)
    norm = math.hypot(x_value, y_value)
    if max_norm <= 0.0 or norm <= max_norm:
        return x_value, y_value
    scale = max_norm / norm
    return x_value * scale, y_value * scale


class MotionActionNode(Node):
    def __init__(self):
        super().__init__('motion_action_node')
        self.callback_group = ReentrantCallbackGroup()
        self.latest_pose = None

        self._declare_parameters()
        self._load_parameters()

        self.velocity_pub = self.create_publisher(
            Float32MultiArray,
            self.velocity_topic,
            10,
        )
        self.relocation_sub = self.create_subscription(
            PoseStamped,
            self.relocation_topic,
            self._relocation_callback,
            10,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            MoveToPose,
            ACTION_NAME,
            execute_callback=self._execute_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info(
            f'Motion action server ready on /{ACTION_NAME}; '
            f'publishing velocity to {self.velocity_topic}'
        )

    def _declare_parameters(self):
        self.declare_parameter('relocation_topic', '/odin1/relocation')
        self.declare_parameter('velocity_topic', '/t0x0101_pid')
        self.declare_parameter('control_frequency_hz', 50.0)
        self.declare_parameter('initial_pose_timeout_sec', 2.0)
        self.declare_parameter('yaw_timeout_sec', 5.0)
        self.declare_parameter('xy_timeout_sec', 5.0)
        self.declare_parameter('yaw_tolerance_deg', 1.0)
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('min_linear_speed', 0.0)
        self.declare_parameter('min_angular_speed', 0.0)
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('yaw_pid.kp', 1.2)
        self.declare_parameter('yaw_pid.ki', 0.0)
        self.declare_parameter('yaw_pid.kd', 0.05)
        self.declare_parameter('x_pid.kp', 0.8)
        self.declare_parameter('x_pid.ki', 0.0)
        self.declare_parameter('x_pid.kd', 0.02)
        self.declare_parameter('y_pid.kp', 0.8)
        self.declare_parameter('y_pid.ki', 0.0)
        self.declare_parameter('y_pid.kd', 0.02)

    def _load_parameters(self):
        self.relocation_topic = self.get_parameter(
            'relocation_topic').value
        self.velocity_topic = self.get_parameter('velocity_topic').value
        self.control_frequency_hz = float(self.get_parameter(
            'control_frequency_hz').value)
        self.initial_pose_timeout_sec = float(self.get_parameter(
            'initial_pose_timeout_sec').value)
        self.yaw_timeout_sec = float(self.get_parameter(
            'yaw_timeout_sec').value)
        self.xy_timeout_sec = float(self.get_parameter(
            'xy_timeout_sec').value)
        self.yaw_tolerance_rad = math.radians(float(self.get_parameter(
            'yaw_tolerance_deg').value))
        self.position_tolerance = float(self.get_parameter(
            'position_tolerance').value)
        self.max_linear_speed = float(self.get_parameter(
            'max_linear_speed').value)
        self.max_angular_speed = float(self.get_parameter(
            'max_angular_speed').value)
        self.min_linear_speed = float(self.get_parameter(
            'min_linear_speed').value)
        self.min_angular_speed = float(self.get_parameter(
            'min_angular_speed').value)
        self.integral_limit = float(self.get_parameter(
            'integral_limit').value)

    def _make_pid(self, prefix):
        return PidController(
            float(self.get_parameter(f'{prefix}.kp').value),
            float(self.get_parameter(f'{prefix}.ki').value),
            float(self.get_parameter(f'{prefix}.kd').value),
            self.integral_limit,
        )

    def _goal_callback(self, goal_request):
        self.get_logger().info(
            'Received goal: '
            f'x={goal_request.x:.3f}, '
            f'y={goal_request.y:.3f}, '
            f'yaw_deg={goal_request.yaw_deg:.3f}'
        )
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request accepted')
        return CancelResponse.ACCEPT

    def _relocation_callback(self, msg):
        pose = msg.pose
        self.latest_pose = Pose2D(
            x=pose.position.x,
            y=pose.position.y,
            yaw_rad=yaw_from_quaternion(pose.orientation),
        )

    def _execute_callback(self, goal_handle):
        goal = goal_handle.request
        result = MoveToPose.Result()

        yaw_pid = self._make_pid('yaw_pid')
        x_pid = self._make_pid('x_pid')
        y_pid = self._make_pid('y_pid')
        target_yaw_rad = math.radians(goal.yaw_deg)
        period = 1.0 / max(self.control_frequency_hz, 1.0)

        wait_status = self._wait_for_pose(goal_handle, goal, period)
        if wait_status == 'canceled':
            result.success = False
            result.message = 'Goal canceled while waiting for relocation pose'
            return result
        if wait_status != 'ready':
            result.success = False
            result.message = 'Timed out waiting for relocation pose'
            goal_handle.abort()
            self._publish_stop()
            return result

        yaw_start_time = time.monotonic()
        last_time = yaw_start_time
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled during yaw phase'
                self._publish_stop()
                return result

            now = time.monotonic()
            dt = now - last_time
            last_time = now
            pose = self.latest_pose
            yaw_error = normalize_angle(target_yaw_rad - pose.yaw_rad)

            if abs(yaw_error) <= self.yaw_tolerance_rad:
                self._publish_stop()
                break

            if now - yaw_start_time > self.yaw_timeout_sec:
                result.success = False
                result.message = 'Yaw phase timed out'
                goal_handle.abort()
                self._publish_stop()
                return result

            cmd_wz = yaw_pid.update(yaw_error, dt)
            cmd_wz = apply_min_output(cmd_wz, self.min_angular_speed)
            cmd_wz = clamp(
                cmd_wz,
                -self.max_angular_speed,
                self.max_angular_speed,
            )
            self._publish_velocity(0.0, 0.0, cmd_wz)
            self._publish_feedback(
                goal_handle,
                goal,
                pose,
                PHASE_YAW,
                yaw_error,
                self._distance_error(goal, pose),
                0.0,
                0.0,
                cmd_wz,
            )
            time.sleep(period)

        yaw_pid.reset()
        x_pid.reset()
        y_pid.reset()
        xy_start_time = time.monotonic()
        last_time = xy_start_time
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled during xy phase'
                self._publish_stop()
                return result

            now = time.monotonic()
            dt = now - last_time
            last_time = now
            pose = self.latest_pose
            dx_map = goal.x - pose.x
            dy_map = goal.y - pose.y
            distance_error = math.hypot(dx_map, dy_map)
            yaw_error = normalize_angle(target_yaw_rad - pose.yaw_rad)

            if distance_error <= self.position_tolerance:
                result.success = True
                result.message = 'Goal reached'
                goal_handle.succeed()
                self._publish_stop()
                return result

            if now - xy_start_time > self.xy_timeout_sec:
                result.success = False
                result.message = 'XY phase timed out'
                goal_handle.abort()
                self._publish_stop()
                return result

            error_x_body, error_y_body = map_error_to_body(
                dx_map,
                dy_map,
                pose.yaw_rad,
            )
            cmd_vx = x_pid.update(error_x_body, dt)
            cmd_vy = y_pid.update(error_y_body, dt)
            cmd_vx = apply_min_output(cmd_vx, self.min_linear_speed)
            cmd_vy = apply_min_output(cmd_vy, self.min_linear_speed)
            cmd_vx, cmd_vy = limit_vector(
                cmd_vx,
                cmd_vy,
                self.max_linear_speed,
            )
            cmd_wz = yaw_pid.update(yaw_error, dt)
            cmd_wz = apply_min_output(cmd_wz, self.min_angular_speed)
            cmd_wz = clamp(
                cmd_wz,
                -self.max_angular_speed,
                self.max_angular_speed,
            )

            self._publish_velocity(cmd_vx, cmd_vy, cmd_wz)
            self._publish_feedback(
                goal_handle,
                goal,
                pose,
                PHASE_XY,
                yaw_error,
                distance_error,
                cmd_vx,
                cmd_vy,
                cmd_wz,
            )
            time.sleep(period)

        result.success = False
        result.message = 'ROS shutdown during goal execution'
        goal_handle.abort()
        self._publish_stop()
        return result

    def _wait_for_pose(self, goal_handle, goal, period):
        start_time = time.monotonic()
        while rclpy.ok() and self.latest_pose is None:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._publish_stop()
                return 'canceled'
            if time.monotonic() - start_time > self.initial_pose_timeout_sec:
                return 'timeout'
            self._publish_feedback(
                goal_handle,
                goal,
                Pose2D(0.0, 0.0, 0.0),
                PHASE_WAITING_FOR_POSE,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
            )
            time.sleep(period)
        if self.latest_pose is None:
            return 'timeout'
        return 'ready'

    def _publish_feedback(
        self,
        goal_handle,
        goal,
        pose,
        phase,
        yaw_error_rad,
        distance_error,
        cmd_vx,
        cmd_vy,
        cmd_wz,
    ):
        feedback = MoveToPose.Feedback()
        feedback.phase = phase
        feedback.current_x = pose.x
        feedback.current_y = pose.y
        feedback.current_yaw_deg = math.degrees(pose.yaw_rad)
        feedback.target_x = goal.x
        feedback.target_y = goal.y
        feedback.target_yaw_deg = goal.yaw_deg
        feedback.yaw_error_deg = math.degrees(yaw_error_rad)
        feedback.distance_error = distance_error
        feedback.cmd_vx = cmd_vx
        feedback.cmd_vy = cmd_vy
        feedback.cmd_wz = cmd_wz
        goal_handle.publish_feedback(feedback)

    def _distance_error(self, goal, pose):
        return math.hypot(goal.x - pose.x, goal.y - pose.y)

    def _publish_velocity(self, vx, vy, wz):
        msg = Float32MultiArray()
        msg.data = [float(vx), float(vy), float(wz)]
        self.velocity_pub.publish(msg)

    def _publish_stop(self):
        self._publish_velocity(0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = MotionActionNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
