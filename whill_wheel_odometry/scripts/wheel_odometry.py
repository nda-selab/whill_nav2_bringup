#!/usr/bin/env python3
"""Convert actual WHILL motor speed feedback into Odometry; no TF broadcast."""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock, ClockType
from rclpy.qos import QoSProfile, ReliabilityPolicy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from nav_msgs.msg import Odometry
from whill_msgs.msg import ModelCr2State

from whill_wheel_odometry.kinematics import DriveModel, WheelOdometry, covariance6


class WheelOdometryNode(Node):
    def __init__(self):
        super().__init__('wheel_odometry')
        defaults = {
            'input_topic': '/whill/states/model_cr2', 'output_topic': '/wheel_odom',
            'diagnostics_topic': '/wheel_odometry/diagnostics',
            'odom_frame': 'odom', 'base_frame': 'base_link',
            'wheel_tread': 0.496, 'left_sign': 1, 'right_sign': 1,
            'left_scale': 1.0, 'right_scale': 1.0,
            'axle_to_base_x': 0.0, 'axle_to_base_y': 0.0,
            'wheel_speed_std': 0.03, 'lateral_speed_std': 0.05,
            'constrain_vertical_velocity': False, 'vertical_speed_std': 0.10,
            'max_wheel_speed': 2.0, 'max_dt': 0.15, 'input_timeout': 0.20,
            'diagnostic_rate': 10.0, 'calibration_confirmed': False,
        }
        for name, default in defaults.items():
            self.declare_parameter(name, default)
        p = {name: self.get_parameter(name).value for name in defaults}
        self.model = DriveModel(**{name: p[name] for name in DriveModel.__dataclass_fields__})
        self.estimator = WheelOdometry(self.model)
        self.max_dt, self.input_timeout = p['max_dt'], p['input_timeout']
        if not all(math.isfinite(v) and v > 0.0 for v in
                   (self.max_dt, self.input_timeout, p['diagnostic_rate'])):
            raise ValueError('Time limits and diagnostic rate must be finite and positive')
        self.odom_frame, self.base_frame = p['odom_frame'], p['base_frame']
        if (not self.odom_frame or not self.base_frame or self.odom_frame == self.base_frame
                or self.odom_frame.startswith('/') or self.base_frame.startswith('/')):
            raise ValueError('Frame IDs must be distinct, nonempty and have no leading slash')
        self.calibration_confirmed = p['calibration_confirmed']
        self.constrain_vertical_velocity = p['constrain_vertical_velocity']
        self.vertical_speed_std = p['vertical_speed_std']
        if not math.isfinite(self.vertical_speed_std) or self.vertical_speed_std <= 0.0:
            raise ValueError('vertical_speed_std must be finite and positive')
        if self.constrain_vertical_velocity and (abs(self.model.axle_to_base_x) > 1e-6
                                               or abs(self.model.axle_to_base_y) > 1e-6):
            raise ValueError('Zero vertical velocity requires the base origin at the axle in XY')
        self.last_received = None
        self.last_stamp_ns = None
        self.previous_wheels = None
        self.last_valid = None
        self.last_raw = None
        self.last_sample_stamp_ns = None
        self.reason = 'waiting for WHILL feedback'
        self.odom_pub = self.create_publisher(Odometry, p['output_topic'], 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, p['diagnostics_topic'], 10)
        self.subscription = self.create_subscription(
            ModelCr2State, p['input_topic'], self.on_state,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
        self.timer = self.create_timer(1.0 / p['diagnostic_rate'],
                                       self.publish_diagnostics, clock=self.steady_clock)
        if not self.calibration_confirmed:
            self.get_logger().warn('Calibration unconfirmed: odometry publication disabled. '
                                   'Verify wheel signs, scale and base geometry before enabling.')
        # Configuration is startup-only; avoid reporting an accepted but unused change.
        from rcl_interfaces.msg import SetParametersResult
        self.add_on_set_parameters_callback(lambda params: SetParametersResult(
            successful=not any(p.name in defaults for p in params),
            reason='Restart the node to change wheel odometry configuration'))

    def invalidate(self, reason):
        if self.last_stamp_ns is not None or self.last_valid is not None:
            self.estimator.invalidate_pose()
        self.last_stamp_ns = None
        self.previous_wheels = None
        self.last_valid = None
        self.reason = reason

    def reject(self, reason):
        self.invalidate(reason)
        self.publish_diagnostics()

    def on_state(self, msg):
        received = time.monotonic()
        previous_received = self.last_received
        self.last_received = received
        stamp = self.get_clock().now()
        self.last_raw = (msg.left_motor_speed, msg.right_motor_speed, msg.power_on, msg.error)
        if not self.calibration_confirmed:
            self.reject('calibration not confirmed')
            return
        if msg.error != 0 or msg.power_on != 1:
            self.reject(f'WHILL power/error: {msg.power_on}/{msg.error}')
            return
        try:
            wheels = self.model.wheel_speeds(msg.left_motor_speed, msg.right_motor_speed)
        except ValueError as exc:
            self.reject(str(exc))
            return
        if stamp.nanoseconds <= 0:
            self.reject('ROS clock not initialized')
            return
        if self.last_sample_stamp_ns is not None and stamp.nanoseconds <= self.last_sample_stamp_ns:
            self.reject('ROS timestamp did not advance; restart after clock reset')
            return
        self.last_sample_stamp_ns = stamp.nanoseconds
        if previous_received is not None and received - previous_received > self.input_timeout:
            self.invalidate('WHILL feedback gap')
            self.publish_diagnostics()
        if self.last_stamp_ns is None:
            self.last_stamp_ns, self.previous_wheels = stamp.nanoseconds, wheels
            self.reason = 'waiting for second consecutive valid sample'
            return
        dt = (stamp.nanoseconds - self.last_stamp_ns) * 1e-9
        if not 0.0 < dt <= self.max_dt:
            self.reject(f'Invalid integration interval: {dt:.3f}s')
            return
        # Trapezoidal velocity integration; no extrapolation over missing samples.
        left, right = ((a+b)/2.0 for a, b in zip(self.previous_wheels, wheels))
        self.estimator.integrate(left, right, dt)
        self.last_stamp_ns, self.previous_wheels = stamp.nanoseconds, wheels
        x, y, yaw = self.estimator.base_pose()
        vx, vy, wz = self.model.base_twist(*wheels)
        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x, odom.pose.pose.position.y = x, y
        odom.pose.pose.orientation.z = math.sin(yaw / 2.0)
        odom.pose.pose.orientation.w = math.cos(yaw / 2.0)
        odom.pose.covariance = covariance6(self.estimator.pose_covariance())
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = vx, vy
        odom.twist.twist.angular.z = wz
        twist_cov = covariance6(self.model.twist_covariance())
        # Optional nonholonomic pseudo-measurement in the BODY frame, not world Z.
        if self.constrain_vertical_velocity:
            twist_cov[14] = self.vertical_speed_std**2
        odom.twist.covariance = twist_cov
        self.odom_pub.publish(odom)
        self.last_valid = received
        self.reason = 'ok' if self.estimator.pose_valid else 'twist valid; integrated pose lost across a gap'

    def publish_diagnostics(self):
        now = time.monotonic()
        age = math.inf if self.last_received is None else now - self.last_received
        if self.last_received is not None and age > self.input_timeout:
            self.invalidate('WHILL feedback stale')
        valid_age = math.inf if self.last_valid is None else now - self.last_valid
        status = DiagnosticStatus()
        status.name = 'whill_wheel_odometry/input'
        status.hardware_id = 'WHILL_CR2'
        if not self.calibration_confirmed:
            status.level, status.message = DiagnosticStatus.ERROR, 'calibration not confirmed'
        elif age > self.input_timeout:
            status.level, status.message = DiagnosticStatus.STALE, self.reason
        elif valid_age > self.input_timeout:
            status.level, status.message = DiagnosticStatus.ERROR, self.reason
        else:
            # Pose invalidity is explicit; EKF should fuse twist only. A safety
            # consumer may choose to latch a stop on WARN until operator reset.
            status.level = DiagnosticStatus.OK if self.estimator.pose_valid else DiagnosticStatus.WARN
            status.message = self.reason
        values = {'input_age_sec': age, 'valid_age_sec': valid_age,
                  'input_timeout_sec': self.input_timeout,
                  'calibration_confirmed': self.calibration_confirmed,
                  'pose_valid': self.estimator.pose_valid,
                  'twist_valid': valid_age <= self.input_timeout and age <= self.input_timeout}
        if self.last_raw is not None:
            values.update(zip(('left_motor_speed_kmh', 'right_motor_speed_kmh', 'power_on', 'error'), self.last_raw))
        status.values = [KeyValue(key=k, value=str(v).lower()) for k, v in values.items()]
        msg = DiagnosticArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.status = [status]
        self.diag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WheelOdometryNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
