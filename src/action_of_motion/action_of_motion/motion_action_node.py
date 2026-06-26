import math
import os
import threading
import time
from dataclasses import dataclass
from datetime import datetime

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
PHASE_POSE = 'pose'
PID_PROFILE_SLOW = 0
PID_PROFILE_FAST = 1
PID_PROFILE_NAMES = {
    PID_PROFILE_SLOW: 'slow',
    PID_PROFILE_FAST: 'fast',
}
PATH_LENGTH_EPSILON = 1e-6


@dataclass
class Pose2D:
    x: float
    y: float
    yaw_rad: float


@dataclass
class PoseControlProfile:
    name: str
    max_linear_speed: float
    max_yaw_angular_speed: float
    min_linear_speed: float
    min_yaw_angular_speed: float
    along_pid: 'PidController'
    cross_pid: 'PidController'
    yaw_pid: 'PidController'


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


def apply_min_vector_output(x_value, y_value, minimum):
    minimum = abs(minimum)
    norm = math.hypot(x_value, y_value)
    if minimum <= 0.0 or norm == 0.0 or norm >= minimum:
        return x_value, y_value
    scale = minimum / norm
    return x_value * scale, y_value * scale


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


def map_velocity_to_body(vx_map, vy_map, yaw_rad):
    cos_yaw = math.cos(yaw_rad)
    sin_yaw = math.sin(yaw_rad)
    vx_body = cos_yaw * vx_map + sin_yaw * vy_map
    vy_body = -sin_yaw * vx_map + cos_yaw * vy_map
    return vx_body, vy_body


def limit_vector(x_value, y_value, max_norm):
    max_norm = abs(max_norm)
    norm = math.hypot(x_value, y_value)
    if max_norm <= 0.0 or norm <= max_norm:
        return x_value, y_value
    scale = max_norm / norm
    return x_value * scale, y_value * scale


def sleep_to_next_tick(next_tick, period):
    next_tick += period
    sleep_sec = next_tick - time.monotonic()
    if sleep_sec > 0.0:
        time.sleep(sleep_sec)
        return next_tick
    return time.monotonic()


class MotionActionNode(Node):
    def __init__(self):
        super().__init__('motion_action_node')
        self.callback_group = ReentrantCallbackGroup()
        self.latest_pose = None

        self._declare_parameters()
        self._load_parameters()
        self.debug_samples = []
        self.debug_record_start_time = None
        self._goal_accepted_time = None
        self._first_velocity_logged = False

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
        self.declare_parameter('feedback_frequency_hz', 20.0)
        self.declare_parameter('lightweight_feedback_frequency_hz', 5.0)
        self.declare_parameter('initial_pose_timeout_sec', 2.0)
        self.declare_parameter('pose_timeout_sec', 10.0)
        self.declare_parameter('brake_duration_sec', 0.3)
        self.declare_parameter('brake_frequency_hz', 50.0)
        self.declare_parameter('enable_debug_plot_files', True)
        self.declare_parameter('debug_output_dir', 'output/pid_debug')
        self.declare_parameter('yaw_tolerance_deg', 1.0)
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('integral_limit', 0.5)
        self._declare_profile_parameters('slow_profile', 0.8)
        self._declare_profile_parameters('fast_profile', 2.0)

    def _declare_profile_parameters(self, prefix, max_linear_speed):
        self.declare_parameter(f'{prefix}.max_linear_speed',
                               max_linear_speed)
        self.declare_parameter(f'{prefix}.max_yaw_angular_speed', 0.4)
        self.declare_parameter(f'{prefix}.min_linear_speed', 0.15)
        self.declare_parameter(f'{prefix}.min_yaw_angular_speed', 0.0)
        self.declare_parameter(f'{prefix}.along_pid.kp', 5.0)
        self.declare_parameter(f'{prefix}.along_pid.ki', 0.01)
        self.declare_parameter(f'{prefix}.along_pid.kd', 0.1)
        self.declare_parameter(f'{prefix}.cross_pid.kp', 5.0)
        self.declare_parameter(f'{prefix}.cross_pid.ki', 0.01)
        self.declare_parameter(f'{prefix}.cross_pid.kd', 0.1)
        self.declare_parameter(f'{prefix}.yaw_pid.kp', 1.0)
        self.declare_parameter(f'{prefix}.yaw_pid.ki', 0.0)
        self.declare_parameter(f'{prefix}.yaw_pid.kd', 0.0)

    def _load_parameters(self):
        self.relocation_topic = self.get_parameter(
            'relocation_topic').value
        self.velocity_topic = self.get_parameter('velocity_topic').value
        self.control_frequency_hz = float(self.get_parameter(
            'control_frequency_hz').value)
        self.feedback_frequency_hz = float(self.get_parameter(
            'feedback_frequency_hz').value)
        self.lightweight_feedback_frequency_hz = float(self.get_parameter(
            'lightweight_feedback_frequency_hz').value)
        self.initial_pose_timeout_sec = float(self.get_parameter(
            'initial_pose_timeout_sec').value)
        self.pose_timeout_sec = float(self.get_parameter(
            'pose_timeout_sec').value)
        self.brake_duration_sec = float(self.get_parameter(
            'brake_duration_sec').value)
        self.brake_frequency_hz = float(self.get_parameter(
            'brake_frequency_hz').value)
        self.enable_debug_plot_files = bool(self.get_parameter(
            'enable_debug_plot_files').value)
        active_feedback_hz = (
            self.feedback_frequency_hz
            if self.enable_debug_plot_files
            else self.lightweight_feedback_frequency_hz
        )
        self.feedback_period_sec = (
            1.0 / active_feedback_hz
            if active_feedback_hz > 0.0
            else 0.0
        )
        self.debug_output_dir = self.get_parameter(
            'debug_output_dir').value
        self.yaw_tolerance_rad = math.radians(float(self.get_parameter(
            'yaw_tolerance_deg').value))
        self.position_tolerance = float(self.get_parameter(
            'position_tolerance').value)
        self.integral_limit = float(self.get_parameter(
            'integral_limit').value)

    def _make_pid(self, prefix):
        return PidController(
            float(self.get_parameter(f'{prefix}.kp').value),
            float(self.get_parameter(f'{prefix}.ki').value),
            float(self.get_parameter(f'{prefix}.kd').value),
            self.integral_limit,
        )

    def _make_profile(self, profile_id):
        name = self._profile_name(profile_id)
        prefix = f'{name}_profile'
        return PoseControlProfile(
            name=name,
            max_linear_speed=float(self.get_parameter(
                f'{prefix}.max_linear_speed').value),
            max_yaw_angular_speed=float(self.get_parameter(
                f'{prefix}.max_yaw_angular_speed').value),
            min_linear_speed=float(self.get_parameter(
                f'{prefix}.min_linear_speed').value),
            min_yaw_angular_speed=float(self.get_parameter(
                f'{prefix}.min_yaw_angular_speed').value),
            along_pid=self._make_pid(f'{prefix}.along_pid'),
            cross_pid=self._make_pid(f'{prefix}.cross_pid'),
            yaw_pid=self._make_pid(f'{prefix}.yaw_pid'),
        )

    def _profile_name(self, profile_id):
        return PID_PROFILE_NAMES.get(profile_id, 'unknown')

    def _goal_callback(self, goal_request):
        if goal_request.pid_profile not in PID_PROFILE_NAMES:
            self.get_logger().warning(
                'Rejecting goal with invalid pid_profile='
                f'{goal_request.pid_profile}'
            )
            return GoalResponse.REJECT

        self.get_logger().info(
            'Received goal: '
            f'x={goal_request.x:.3f}, '
            f'y={goal_request.y:.3f}, '
            f'yaw_deg={goal_request.yaw_deg:.3f}, '
            f'pid_profile={self._profile_name(goal_request.pid_profile)}'
        )
        self._goal_accepted_time = time.monotonic()
        self.get_logger().info('MoveToPose latency: goal accepted')
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
        self._start_debug_recording()
        self._first_velocity_logged = False
        execute_start_time = time.monotonic()
        if self._goal_accepted_time is not None:
            accept_to_execute_ms = (
                execute_start_time - self._goal_accepted_time
            ) * 1000.0
            self.get_logger().info(
                'MoveToPose latency: execute callback started '
                f'{accept_to_execute_ms:.1f} ms after goal accepted'
            )
        else:
            self.get_logger().info(
                'MoveToPose latency: execute callback started'
            )

        target_yaw_rad = math.radians(goal.yaw_deg)
        period = 1.0 / max(self.control_frequency_hz, 1.0)

        wait_status = self._wait_for_pose(goal_handle, goal, period)
        if wait_status == 'canceled':
            result.success = False
            result.message = 'Goal canceled while waiting for relocation pose'
            return self._finish_result(goal, result)
        if wait_status != 'ready':
            result.success = False
            result.message = 'Timed out waiting for relocation pose'
            goal_handle.abort()
            self._brake()
            return self._finish_result(goal, result)

        profile = self._make_profile(goal.pid_profile)
        return self._execute_simultaneous(
            goal_handle,
            goal,
            result,
            target_yaw_rad,
            period,
            profile,
        )

    def _execute_simultaneous(
        self,
        goal_handle,
        goal,
        result,
        target_yaw_rad,
        period,
        profile,
    ):
        start_pose = self.latest_pose
        path_dx = goal.x - start_pose.x
        path_dy = goal.y - start_pose.y
        path_length = math.hypot(path_dx, path_dy)
        if path_length > PATH_LENGTH_EPSILON:
            path_unit_x = path_dx / path_length
            path_unit_y = path_dy / path_length
            cross_unit_x = -path_unit_y
            cross_unit_y = path_unit_x
        else:
            path_unit_x = 0.0
            path_unit_y = 0.0
            cross_unit_x = 0.0
            cross_unit_y = 0.0

        pose_start_time = time.monotonic()
        last_time = pose_start_time
        next_tick = pose_start_time
        last_feedback_time = None

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled during pose phase'
                self._brake()
                return self._finish_result(goal, result)

            now = time.monotonic()
            dt = now - last_time
            last_time = now
            pose = self.latest_pose
            dx_map = goal.x - pose.x
            dy_map = goal.y - pose.y
            distance_error = math.hypot(dx_map, dy_map)
            yaw_error = normalize_angle(target_yaw_rad - pose.yaw_rad)

            if (
                distance_error <= self.position_tolerance
                and abs(yaw_error) <= self.yaw_tolerance_rad
            ):
                result.success = True
                result.message = 'Goal reached'
                goal_handle.succeed()
                self._brake()
                return self._finish_result(goal, result)

            if now - pose_start_time > self.pose_timeout_sec:
                result.success = False
                result.message = 'Pose phase timed out'
                goal_handle.abort()
                self._brake()
                return self._finish_result(goal, result)

            start_to_pose_x = pose.x - start_pose.x
            start_to_pose_y = pose.y - start_pose.y
            goal_error_x = goal.x - pose.x
            goal_error_y = goal.y - pose.y
            error_x_body, error_y_body = map_error_to_body(
                goal_error_x,
                goal_error_y,
                pose.yaw_rad,
            )
            if distance_error <= self.position_tolerance:
                cmd_vx_map = 0.0
                cmd_vy_map = 0.0
            elif path_length > PATH_LENGTH_EPSILON:
                along_error = (
                    goal_error_x * path_unit_x
                    + goal_error_y * path_unit_y
                )
                cross_position = (
                    start_to_pose_x * cross_unit_x
                    + start_to_pose_y * cross_unit_y
                )
                cross_error = -cross_position
                cmd_along = profile.along_pid.update(along_error, dt)
                cmd_cross = profile.cross_pid.update(cross_error, dt)
                cmd_vx_map = (
                    cmd_along * path_unit_x
                    + cmd_cross * cross_unit_x
                )
                cmd_vy_map = (
                    cmd_along * path_unit_y
                    + cmd_cross * cross_unit_y
                )
            else:
                cmd_vx_map = 0.0
                cmd_vy_map = 0.0

            cmd_wz = profile.yaw_pid.update(yaw_error, dt)
            cmd_wz = apply_min_output(
                cmd_wz,
                profile.min_yaw_angular_speed,
            )
            cmd_wz = clamp(
                cmd_wz,
                -profile.max_yaw_angular_speed,
                profile.max_yaw_angular_speed,
            )
            compensated_yaw = pose.yaw_rad + cmd_wz * dt * 0.5
            cmd_vx, cmd_vy = map_velocity_to_body(
                cmd_vx_map,
                cmd_vy_map,
                compensated_yaw,
            )
            cmd_vx, cmd_vy = apply_min_vector_output(
                cmd_vx,
                cmd_vy,
                profile.min_linear_speed,
            )
            cmd_vx, cmd_vy = limit_vector(
                cmd_vx,
                cmd_vy,
                profile.max_linear_speed,
            )

            self._publish_velocity(cmd_vx, cmd_vy, cmd_wz)
            feedback_now = time.monotonic()
            if self._should_publish_feedback(
                feedback_now,
                last_feedback_time,
            ):
                last_feedback_time = feedback_now
                self._publish_feedback(
                    goal_handle,
                    goal,
                    pose,
                    PHASE_POSE,
                    yaw_error,
                    distance_error,
                    error_x_body,
                    error_y_body,
                    cmd_vx,
                    cmd_vy,
                    cmd_wz,
                )
            next_tick = sleep_to_next_tick(next_tick, period)

        result.success = False
        result.message = 'ROS shutdown during goal execution'
        goal_handle.abort()
        self._brake()
        return self._finish_result(goal, result)

    def _wait_for_pose(self, goal_handle, goal, period):
        start_time = time.monotonic()
        next_tick = start_time
        last_feedback_time = None
        while rclpy.ok() and self.latest_pose is None:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._brake()
                return 'canceled'
            if time.monotonic() - start_time > self.initial_pose_timeout_sec:
                return 'timeout'
            feedback_now = time.monotonic()
            if self._should_publish_feedback(
                feedback_now,
                last_feedback_time,
            ):
                last_feedback_time = feedback_now
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
                    0.0,
                    0.0,
                )
            next_tick = sleep_to_next_tick(next_tick, period)
        if self.latest_pose is None:
            return 'timeout'
        return 'ready'

    def _should_publish_feedback(self, now, last_feedback_time):
        if self.feedback_period_sec <= 0.0:
            return False
        return (
            last_feedback_time is None
            or now - last_feedback_time >= self.feedback_period_sec
        )

    def _publish_feedback(
        self,
        goal_handle,
        goal,
        pose,
        phase,
        yaw_error_rad,
        distance_error,
        error_x_body,
        error_y_body,
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
        if self.enable_debug_plot_files:
            feedback.cmd_vx = cmd_vx
            feedback.cmd_vy = cmd_vy
            feedback.cmd_wz = cmd_wz
        goal_handle.publish_feedback(feedback)
        if self.enable_debug_plot_files:
            self._record_debug_values(
                phase,
                pose,
                goal,
                yaw_error_rad,
                distance_error,
                error_x_body,
                error_y_body,
                cmd_vx,
                cmd_vy,
                cmd_wz,
            )

    def _record_debug_values(
        self,
        phase,
        pose,
        goal,
        yaw_error_rad,
        distance_error,
        error_x_body,
        error_y_body,
        cmd_vx,
        cmd_vy,
        cmd_wz,
    ):
        phase_ids = {
            PHASE_WAITING_FOR_POSE: 0.0,
            PHASE_POSE: 1.0,
        }
        values = {
            'phase_id': phase_ids.get(phase, -1.0),
            'pid_profile_id': float(goal.pid_profile),
            'current_x': pose.x,
            'current_y': pose.y,
            'current_yaw_deg': math.degrees(pose.yaw_rad),
            'target_x': goal.x,
            'target_y': goal.y,
            'target_yaw_deg': goal.yaw_deg,
            'yaw_error_deg': math.degrees(yaw_error_rad),
            'distance_error': distance_error,
            'error_x_body': error_x_body,
            'error_y_body': error_y_body,
            'cmd_vx': cmd_vx,
            'cmd_vy': cmd_vy,
            'cmd_wz': cmd_wz,
            'cmd_linear_speed': math.hypot(cmd_vx, cmd_vy),
        }
        self._record_debug_sample(values)

    def _start_debug_recording(self):
        self.debug_samples = []
        self.debug_record_start_time = time.monotonic()

    def _record_debug_sample(self, values):
        if not self.enable_debug_plot_files:
            return
        if self.debug_record_start_time is None:
            return

        sample = {'time_sec': time.monotonic() - self.debug_record_start_time}
        sample.update(values)
        self.debug_samples.append(sample)

    def _finish_result(self, goal, result):
        self.get_logger().info(
            'MoveToPose latency: result finish, '
            f'success={result.success}, message="{result.message}"'
        )
        if self.enable_debug_plot_files and len(self.debug_samples) >= 2:
            debug_samples = [sample.copy() for sample in self.debug_samples]
            thread = threading.Thread(
                target=self._save_debug_plot,
                args=(goal, result, debug_samples),
                daemon=True,
            )
            thread.start()
        return result

    def _save_debug_plot(self, goal, result, debug_samples):
        if not self.enable_debug_plot_files or len(debug_samples) < 2:
            return

        try:
            import matplotlib
            matplotlib.use('Agg')
            import matplotlib.pyplot as plt
        except ImportError as exc:
            self.get_logger().warning(
                'Cannot save PID debug plot because matplotlib is missing: '
                f'{exc}. Install python3-matplotlib.'
            )
            return

        os.makedirs(self.debug_output_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        status = 'success' if result.success else 'failed'
        profile_name = self._profile_name(goal.pid_profile)
        filename = (
            f'move_to_pose_{timestamp}_{status}_'
            f'{profile_name}_'
            f'x{goal.x:.2f}_y{goal.y:.2f}_yaw{goal.yaw_deg:.1f}.png'
        )
        filepath = os.path.join(self.debug_output_dir, filename)

        times = self._debug_series(debug_samples, 'time_sec')
        distance_errors = self._debug_series(
            debug_samples, 'distance_error')
        yaw_errors = self._debug_series(debug_samples, 'yaw_error_deg')
        error_x_body = self._debug_series(debug_samples, 'error_x_body')
        error_y_body = self._debug_series(debug_samples, 'error_y_body')
        cmd_vx = self._debug_series(debug_samples, 'cmd_vx')
        cmd_vy = self._debug_series(debug_samples, 'cmd_vy')
        cmd_linear_speed = self._debug_series(
            debug_samples, 'cmd_linear_speed')
        cmd_wz = self._debug_series(debug_samples, 'cmd_wz')
        phase_ids = self._debug_series(debug_samples, 'phase_id')
        fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
        fig.suptitle(
            'MoveToPose PID Debug\n'
            f'target=({goal.x:.3f}, {goal.y:.3f}, {goal.yaw_deg:.3f}deg), '
            f'pid_profile={profile_name}, '
            f'result={result.message}'
        )

        axes[0].plot(times, distance_errors, label='distance_error[m]')
        axes[0].plot(times, yaw_errors, label='yaw_error[deg]')
        axes[0].set_ylabel('error')
        axes[0].legend(loc='best')
        axes[0].grid(True)

        axes[1].plot(times, error_x_body, label='error_x_body[m]')
        axes[1].plot(times, error_y_body, label='error_y_body[m]')
        axes[1].set_ylabel('body error')
        axes[1].legend(loc='best')
        axes[1].grid(True)

        axes[2].plot(times, cmd_vx, label='cmd_vx')
        axes[2].plot(times, cmd_vy, label='cmd_vy')
        axes[2].plot(times, cmd_linear_speed, label='cmd_linear_speed')
        axes[2].set_ylabel('linear cmd')
        axes[2].legend(loc='best')
        axes[2].grid(True)

        axes[3].plot(times, cmd_wz, label='cmd_wz')
        axes[3].plot(times, phase_ids, label='phase_id')
        axes[3].set_ylabel('angular/phase')
        axes[3].set_xlabel('time [s]')
        axes[3].legend(loc='best')
        axes[3].grid(True)

        fig.tight_layout()
        fig.savefig(filepath, dpi=140)
        plt.close(fig)
        self.get_logger().info(f'Saved PID debug plot: {filepath}')

    def _debug_series(self, debug_samples, name):
        return [sample[name] for sample in debug_samples]

    def _distance_error(self, goal, pose):
        return math.hypot(goal.x - pose.x, goal.y - pose.y)

    def _publish_velocity(self, vx, vy, wz):
        msg = Float32MultiArray()
        msg.data = [float(vx), float(vy), float(wz)]
        self.velocity_pub.publish(msg)
        if not self._first_velocity_logged:
            self._first_velocity_logged = True
            if self.debug_record_start_time is not None:
                elapsed_ms = (
                    time.monotonic() - self.debug_record_start_time
                ) * 1000.0
                self.get_logger().info(
                    'MoveToPose latency: first velocity published '
                    f'{elapsed_ms:.1f} ms after execute start'
                )

    def _publish_stop(self):
        self._publish_velocity(0.0, 0.0, 0.0)

    def _brake(self):
        period = 1.0 / max(self.brake_frequency_hz, 1.0)
        end_time = time.monotonic() + max(self.brake_duration_sec, 0.0)
        next_tick = time.monotonic()
        self._publish_stop()
        while time.monotonic() < end_time:
            self._publish_stop()
            next_tick = sleep_to_next_tick(next_tick, period)


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
