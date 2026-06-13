import math
import os
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
        self.debug_samples = []
        self.debug_record_start_time = None

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
        self.declare_parameter('brake_duration_sec', 0.3)
        self.declare_parameter('brake_frequency_hz', 50.0)
        self.declare_parameter('enable_debug_plot_files', True)
        self.declare_parameter('debug_output_dir', 'output/pid_debug')
        self.declare_parameter('yaw_tolerance_deg', 1.0)
        self.declare_parameter('position_tolerance', 0.03)
        self.declare_parameter('max_linear_speed', 0.25)
        self.declare_parameter('max_angular_speed', 0.5)
        self.declare_parameter('min_linear_speed', 0.2)
        self.declare_parameter('min_angular_speed', 0.2)
        self.declare_parameter('integral_limit', 0.5)
        self.declare_parameter('yaw_pid.kp', 1.2)
        self.declare_parameter('yaw_pid.ki', 0.0)
        self.declare_parameter('yaw_pid.kd', 0.05)
        self.declare_parameter('xy_yaw_pid.kp', 0.6)
        self.declare_parameter('xy_yaw_pid.ki', 0.0)
        self.declare_parameter('xy_yaw_pid.kd', 0.0)
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
        self.brake_duration_sec = float(self.get_parameter(
            'brake_duration_sec').value)
        self.brake_frequency_hz = float(self.get_parameter(
            'brake_frequency_hz').value)
        self.enable_debug_plot_files = bool(self.get_parameter(
            'enable_debug_plot_files').value)
        self.debug_output_dir = self.get_parameter(
            'debug_output_dir').value
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
        self._start_debug_recording()

        yaw_pid = self._make_pid('yaw_pid')
        xy_yaw_pid = self._make_pid('xy_yaw_pid')
        x_pid = self._make_pid('x_pid')
        y_pid = self._make_pid('y_pid')
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

        yaw_start_time = time.monotonic()
        last_time = yaw_start_time
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled during yaw phase'
                self._brake()
                return self._finish_result(goal, result)

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
                self._brake()
                return self._finish_result(goal, result)

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
                0.0,
                0.0,
                cmd_wz,
            )
            time.sleep(period)

        xy_yaw_pid.reset()
        x_pid.reset()
        y_pid.reset()
        xy_start_time = time.monotonic()
        last_time = xy_start_time
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.success = False
                result.message = 'Goal canceled during xy phase'
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

            if distance_error <= self.position_tolerance:
                result.success = True
                result.message = 'Goal reached'
                goal_handle.succeed()
                self._brake()
                return self._finish_result(goal, result)

            if now - xy_start_time > self.xy_timeout_sec:
                result.success = False
                result.message = 'XY phase timed out'
                goal_handle.abort()
                self._brake()
                return self._finish_result(goal, result)

            error_x_body, error_y_body = map_error_to_body(
                dx_map,
                dy_map,
                pose.yaw_rad,
            )
            cmd_vx = x_pid.update(error_x_body, dt)
            cmd_vy = y_pid.update(error_y_body, dt)
            cmd_vx, cmd_vy = apply_min_vector_output(
                cmd_vx,
                cmd_vy,
                self.min_linear_speed,
            )
            cmd_vx, cmd_vy = limit_vector(
                cmd_vx,
                cmd_vy,
                self.max_linear_speed,
            )
            cmd_wz = xy_yaw_pid.update(yaw_error, dt)
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
                error_x_body,
                error_y_body,
                cmd_vx,
                cmd_vy,
                cmd_wz,
            )
            time.sleep(period)

        result.success = False
        result.message = 'ROS shutdown during goal execution'
        goal_handle.abort()
        self._brake()
        return self._finish_result(goal, result)

    def _wait_for_pose(self, goal_handle, goal, period):
        start_time = time.monotonic()
        while rclpy.ok() and self.latest_pose is None:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self._brake()
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
        feedback.cmd_vx = cmd_vx
        feedback.cmd_vy = cmd_vy
        feedback.cmd_wz = cmd_wz
        goal_handle.publish_feedback(feedback)
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
            PHASE_YAW: 1.0,
            PHASE_XY: 2.0,
        }
        values = {
            'phase_id': phase_ids.get(phase, -1.0),
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
        self._save_debug_plot(goal, result)
        return result

    def _save_debug_plot(self, goal, result):
        if not self.enable_debug_plot_files or len(self.debug_samples) < 2:
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
        filename = (
            f'move_to_pose_{timestamp}_{status}_'
            f'x{goal.x:.2f}_y{goal.y:.2f}_yaw{goal.yaw_deg:.1f}.png'
        )
        filepath = os.path.join(self.debug_output_dir, filename)

        times = self._debug_series('time_sec')
        fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
        fig.suptitle(
            'MoveToPose PID Debug\n'
            f'target=({goal.x:.3f}, {goal.y:.3f}, {goal.yaw_deg:.3f}deg), '
            f'result={result.message}'
        )

        axes[0].plot(times, self._debug_series('distance_error'),
                     label='distance_error[m]')
        axes[0].plot(times, self._debug_series('yaw_error_deg'),
                     label='yaw_error[deg]')
        axes[0].set_ylabel('error')
        axes[0].legend(loc='best')
        axes[0].grid(True)

        axes[1].plot(times, self._debug_series('error_x_body'),
                     label='error_x_body[m]')
        axes[1].plot(times, self._debug_series('error_y_body'),
                     label='error_y_body[m]')
        axes[1].set_ylabel('body error')
        axes[1].legend(loc='best')
        axes[1].grid(True)

        axes[2].plot(times, self._debug_series('cmd_vx'),
                     label='cmd_vx')
        axes[2].plot(times, self._debug_series('cmd_vy'),
                     label='cmd_vy')
        axes[2].plot(times, self._debug_series('cmd_linear_speed'),
                     label='cmd_linear_speed')
        axes[2].set_ylabel('linear cmd')
        axes[2].legend(loc='best')
        axes[2].grid(True)

        axes[3].plot(times, self._debug_series('cmd_wz'),
                     label='cmd_wz')
        axes[3].plot(times, self._debug_series('phase_id'),
                     label='phase_id')
        axes[3].set_ylabel('angular/phase')
        axes[3].set_xlabel('time [s]')
        axes[3].legend(loc='best')
        axes[3].grid(True)

        fig.tight_layout()
        fig.savefig(filepath, dpi=140)
        plt.close(fig)
        self.get_logger().info(f'Saved PID debug plot: {filepath}')

    def _debug_series(self, name):
        return [sample[name] for sample in self.debug_samples]

    def _distance_error(self, goal, pose):
        return math.hypot(goal.x - pose.x, goal.y - pose.y)

    def _publish_velocity(self, vx, vy, wz):
        msg = Float32MultiArray()
        msg.data = [float(vx), float(vy), float(wz)]
        self.velocity_pub.publish(msg)

    def _publish_stop(self):
        self._publish_velocity(0.0, 0.0, 0.0)

    def _brake(self):
        period = 1.0 / max(self.brake_frequency_hz, 1.0)
        end_time = time.monotonic() + max(self.brake_duration_sec, 0.0)
        self._publish_stop()
        while time.monotonic() < end_time:
            self._publish_stop()
            time.sleep(period)


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
