#!/usr/bin/env python3

import math
import time

from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time


def diagnostic_level(status):
    # ROS 2 Humble may represent DiagnosticStatus.level as bytes.
    level = status.level
    return int.from_bytes(level, 'little') if isinstance(level, bytes) else level


def require_healthy(gate, context):
    reasons = gate.failure_reasons()
    if reasons:
        raise RuntimeError(f'{context}: {"; ".join(reasons)}')


class UrgHealthGate:
    """Prevent continuing when either processed vertical URG is stale/bad."""

    def __init__(self, node, timeout=1.5):
        self.last_at = None
        self.statuses = {}
        self.timeout = timeout
        self.subscription = node.create_subscription(
            DiagnosticArray, '/urg_obstacle_filter/status', self.callback, 10)

    def callback(self, msg):
        self.last_at = time.monotonic()
        self.statuses = {
            status.name.rsplit('/', 1)[-1]: status
            for status in msg.status
            if status.name in ('urg_obstacle_filter/left', 'urg_obstacle_filter/right')
        }

    def failure_reasons(self):
        if self.last_at is None:
            return ['URG: /urg_obstacle_filter/status not received']
        reasons = []
        age = time.monotonic() - self.last_at
        if age > self.timeout:
            reasons.append(f'URG: diagnostics stale (age={age:.3f}s, limit={self.timeout:.3f}s)')
        for side in ('left', 'right'):
            status = self.statuses.get(side)
            if status is None:
                reasons.append(f'URG {side}: diagnostic missing')
            elif diagnostic_level(status) != 0:
                reasons.append(f'URG {side}: level={diagnostic_level(status)}, message={status.message!r}')
        return reasons

    def healthy(self):
        return not self.failure_reasons()


class LocalizationHealthGate:
    def __init__(self, node, timeout=1.5):
        self.timeout = timeout
        self.last_received = None
        self.valid = False
        self.invalid_reason = 'alignment not received'
        self.subscription = node.create_subscription(
            DiagnosticArray, '/alignment_status', self.callback, 10)

    def callback(self, msg):
        for status in msg.status:
            if status.name != 'lidar_localization_ros2/alignment':
                continue
            values = {item.key: item.value for item in status.values}
            self.valid = (
                diagnostic_level(status) == 0 and status.message == 'ok'
                and values.get('has_converged') == 'true'
                and values.get('map_received') == 'true'
                and values.get('initialpose_received') == 'true')
            self.invalid_reason = (
                f'level={diagnostic_level(status)}, message={status.message!r}, '
                f'has_converged={values.get("has_converged", "missing")}, '
                f'map_received={values.get("map_received", "missing")}, '
                f'initialpose_received={values.get("initialpose_received", "missing")}')
            self.last_received = time.monotonic()
            return

    def failure_reasons(self):
        if self.last_received is None:
            return ['localization: /alignment_status alignment not received']
        reasons = []
        age = time.monotonic() - self.last_received
        if age > self.timeout:
            reasons.append(f'localization: diagnostics stale (age={age:.3f}s, limit={self.timeout:.3f}s)')
        if not self.valid:
            reasons.append(f'localization: {self.invalid_reason}')
        return reasons

    def healthy(self):
        return not self.failure_reasons()


class StampedInputGate:
    """Check reception, source time, monotonicity and message content."""

    def __init__(self, node, message_type, topic, validator, timeout=0.2):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('Input timeout must be finite and positive')
        self.node, self.topic, self.validator, self.timeout = node, topic, validator, timeout
        self.last_received = self.last_stamp = None
        self.invalid_reason = 'not received'
        self.subscription = node.create_subscription(
            message_type, topic, self.callback, qos_profile_sensor_data)

    def callback(self, msg):
        now = time.monotonic()
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        reason = self.validator(msg)
        if stamp <= 0:
            reason = 'invalid timestamp'
        elif self.last_stamp is not None and stamp <= self.last_stamp:
            reason = 'timestamp did not advance'
        elif not 0 <= (self.node.get_clock().now().nanoseconds - stamp) * 1e-9 <= self.timeout:
            reason = 'source timestamp outside freshness window'
        if reason is not None:
            self.invalid_reason = reason
            self.last_received = None
            return
        self.last_stamp, self.last_received = stamp, now
        self.invalid_reason = None

    def failure_reasons(self):
        if self.last_received is None or self.last_stamp is None:
            return [f'{self.topic}: {self.invalid_reason}']
        reasons = []
        age = time.monotonic() - self.last_received
        source_age = (self.node.get_clock().now().nanoseconds - self.last_stamp) * 1e-9
        if age > self.timeout:
            reasons.append(f'{self.topic}: reception stale ({age:.3f}s)')
        if not 0 <= source_age <= self.timeout:
            reasons.append(f'{self.topic}: source timestamp outside freshness window ({source_age:.3f}s)')
        return reasons

    def healthy(self):
        return not self.failure_reasons()


class ImuHealthGate(StampedInputGate):
    def __init__(self, node, topic='/ouster/imu', timeout=0.2,
                 frame='os_imu', require_orientation=False):
        def validate(msg):
            g, a, q = msg.angular_velocity, msg.linear_acceleration, msg.orientation
            if msg.header.frame_id != frame:
                return f'unexpected frame {msg.header.frame_id!r}'
            if not all(math.isfinite(v) for v in (g.x, g.y, g.z, a.x, a.y, a.z)):
                return 'nonfinite IMU measurement'
            for cov in (msg.angular_velocity_covariance, msg.linear_acceleration_covariance):
                if not all(math.isfinite(v) for v in cov) or any(cov[i] < 0 for i in (0, 4, 8)):
                    return 'gyro or acceleration covariance invalid/unavailable'
            if require_orientation:
                norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
                if (not math.isfinite(norm) or abs(norm - 1.0) > 0.05
                        or not all(math.isfinite(msg.orientation_covariance[i])
                                   and msg.orientation_covariance[i] > 0 for i in (0, 4, 8))):
                    return 'invalid orientation or orientation covariance'
            return None
        super().__init__(node, Imu, topic, validate, timeout)


class WheelOdometryHealthGate:
    """Require fresh wheel twist AND healthy driver diagnostics; WARN blocks motion."""

    def __init__(self, node, timeout=0.20, diagnostic_timeout=0.30):
        if not math.isfinite(diagnostic_timeout) or diagnostic_timeout <= 0:
            raise ValueError('Diagnostic timeout must be finite and positive')
        self.node = node
        self.timeout, self.diagnostic_timeout = timeout, diagnostic_timeout
        self.last_received = self.last_stamp = None
        self.invalid_reason = 'diagnostics not received'

        def validate(msg):
            t = msg.twist.twist
            if msg.header.frame_id != 'odom' or msg.child_frame_id != 'base_link':
                return 'unexpected odometry frames'
            if not all(math.isfinite(v) for v in (t.linear.x, t.linear.y, t.angular.z)):
                return 'nonfinite wheel twist'
            if not all(math.isfinite(msg.twist.covariance[i]) and msg.twist.covariance[i] > 0
                       for i in (0, 7, 35)):
                return 'invalid wheel twist covariance'
            return None

        self.odom = StampedInputGate(node, Odometry, '/wheel_odom', validate, timeout)
        self.subscription = node.create_subscription(
            DiagnosticArray, '/wheel_odometry/diagnostics', self.callback, 10)

    def callback(self, msg):
        statuses = [s for s in msg.status if s.name == 'whill_wheel_odometry/input']
        if not statuses:
            return
        self.last_received = time.monotonic()
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        now_ns = self.node.get_clock().now().nanoseconds
        if (len(statuses) != 1 or stamp <= 0 or
                (self.last_stamp is not None and stamp < self.last_stamp) or
                not 0 <= (now_ns - stamp)*1e-9 <= self.diagnostic_timeout):
            self.invalid_reason = 'invalid or stale diagnostics timestamp/status'
            return
        self.last_stamp = stamp
        status = statuses[0]
        v = {item.key: item.value for item in status.values}
        try:
            ages = float(v['input_age_sec']), float(v['valid_age_sec'])
            valid_ages = all(math.isfinite(a) and 0 <= a <= self.timeout for a in ages)
        except (ValueError, KeyError, TypeError):
            valid_ages = False
        valid = (diagnostic_level(status) == 0 and valid_ages
                 and all(v.get(k) == 'true' for k in ('calibration_confirmed', 'twist_valid', 'pose_valid'))
                 and v.get('power_on') == '1' and v.get('error') == '0')
        self.invalid_reason = None if valid else f'level={diagnostic_level(status)}, {status.message}; {v}'

    def failure_reasons(self):
        reasons = self.odom.failure_reasons()
        prefix = '/wheel_odometry/diagnostics'
        if self.last_received is None or self.last_stamp is None:
            return reasons + [f'{prefix}: {self.invalid_reason}']
        elapsed = time.monotonic() - self.last_received
        source_age = (self.node.get_clock().now().nanoseconds - self.last_stamp)*1e-9
        if elapsed > self.diagnostic_timeout or not 0 <= source_age <= self.diagnostic_timeout:
            reasons.append(f'{prefix}: stale diagnostics')
        if self.invalid_reason:
            reasons.append(f'{prefix}: {self.invalid_reason}')
        # /wheel_odom independently checks actual feedback continuity. Do not
        # extrapolate an old diagnostic age past newer valid wheel messages.
        return reasons

    def healthy(self):
        return not self.failure_reasons()


class RobotMotionMonitor:
    def __init__(self, node, timeout=0.5, linear_threshold=0.02,
                 angular_threshold=0.05, stable_time=0.5):
        self.node = node
        self.timeout = timeout
        self.linear_threshold, self.angular_threshold = linear_threshold, angular_threshold
        self.stable_time = stable_time
        self.last_received = self.last_stamp = self.stationary_since = None
        self.linear_speed = self.angular_speed = math.inf
        self.invalid_reason = '/odom_nav not received'
        self.subscription = node.create_subscription(
            Odometry, '/odom_nav', self.callback, qos_profile_sensor_data)

    def callback(self, msg):
        now = time.monotonic()
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        twist = msg.twist.twist
        values = (twist.linear.x, twist.linear.y, twist.linear.z,
                  twist.angular.x, twist.angular.y, twist.angular.z)
        # Check all six components for 3-D EKF; preserve the previous API.
        if (msg.child_frame_id != 'base_link' or not all(math.isfinite(v) for v in values)
                or stamp <= 0):
            self.stationary_since = self.last_received = None
            self.invalid_reason = (f'invalid /odom_nav (child_frame_id={msg.child_frame_id!r}, '
                                   f'stamp_ns={stamp}, velocity={values})')
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.stationary_since = self.last_received = None
            self.invalid_reason = f'/odom_nav timestamp did not advance (stamp_ns={stamp}, previous_ns={self.last_stamp})'
            return
        if self.last_received is None or now - self.last_received > self.timeout:
            self.stationary_since = None
        self.last_stamp, self.last_received, self.invalid_reason = stamp, now, None
        self.linear_speed = math.hypot(*values[:3])
        self.angular_speed = math.hypot(*values[3:])
        age = (self.node.get_clock().now().nanoseconds - stamp)*1e-9
        if (0 <= age <= self.timeout and self.linear_speed <= self.linear_threshold
                and self.angular_speed <= self.angular_threshold):
            if self.stationary_since is None:
                self.stationary_since = now
        else:
            self.stationary_since = None

    def failure_reasons(self):
        if self.last_received is None or self.last_stamp is None:
            return [f'odometry: {self.invalid_reason}']
        age = (self.node.get_clock().now().nanoseconds - self.last_stamp)*1e-9
        reasons = []
        received_age = time.monotonic() - self.last_received
        if received_age > self.timeout:
            reasons.append(f'odometry: /odom_nav reception stale (age={received_age:.3f}s, limit={self.timeout:.3f}s)')
        if not 0 <= age <= self.timeout:
            reasons.append(f'odometry: /odom_nav source timestamp outside freshness window '
                           f'(age={age:.3f}s, allowed=0..{self.timeout:.3f}s)')
        return reasons

    def healthy(self):
        return not self.failure_reasons()

    def reset_stop_confirmation(self):
        self.stationary_since = None

    def stopped(self):
        return (self.healthy() and self.stationary_since is not None
                and time.monotonic() - self.stationary_since >= self.stable_time)


class RouteHealthGate:
    def __init__(self, urg_gate, localization_gate, motion, watchdog, extra_gates=()):
        self.urg_gate, self.localization_gate = urg_gate, localization_gate
        self.motion, self.watchdog = motion, watchdog
        self.extra_gates = tuple(extra_gates)

    def healthy(self):
        return not self.failure_reasons()

    def failure_reasons(self):
        reasons = []
        for gate in (self.urg_gate, self.localization_gate, self.motion, *self.extra_gates):
            reasons.extend(gate.failure_reasons())
        if not self.watchdog.active:
            reasons.append('watchdog: inactive')
        if self.watchdog.fault is not None:
            reasons.append(f'watchdog: {self.watchdog.fault}')
        return reasons


def localization_input_gates(node, mode, *, require_filtered_imu=False):
    """Explicitly select checks; do not infer mode from topic discovery."""
    if mode not in ('slam', 'ekf'):
        raise ValueError(f'Unsupported localization mode: {mode!r}')
    gates = [ImuHealthGate(node)]
    if mode == 'ekf':
        gates.append(WheelOdometryHealthGate(node))
        if require_filtered_imu:
            gates.append(ImuHealthGate(node, topic='/imu/data', require_orientation=True))
    elif require_filtered_imu:
        raise ValueError('Filtered IMU monitoring is only configured for EKF')
    return tuple(gates)
