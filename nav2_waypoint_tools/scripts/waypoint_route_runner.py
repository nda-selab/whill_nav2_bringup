#!/usr/bin/env python3
"""Validate a saved waypoint route; issue Nav2 actions only with --execute.

Position-only intermediate goals; optional required yaw is handled as a separate
Nav2 Spin action after the XY target has been reached. No forced motion on load.
"""
import argparse
import json
import uuid
from datetime import datetime, timezone
import math
from pathlib import Path
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import String
from rclpy.duration import Duration
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from trajectory_waypoint_navigator import load_grid_map, load_footprint
from waypoint_route_core import load_route, route_heading, filter_unsafe_route
from route_watchdog_client import WatchdogClient
from route_health_monitor import (
    LocalizationHealthGate, RobotMotionMonitor, RouteHealthGate,
    UrgHealthGate, require_healthy, localization_input_gates,
)

from nav2_msgs.msg import SpeedLimit


class WaypointEventPublisher:
    """Publish one structured announcement event per route state transition."""

    def __init__(self, node):
        self.publisher = node.create_publisher(String, '/navigation/waypoint_event', 10)
        self.mission_id = uuid.uuid4().hex
        self.sequence = 0

    def send(self, event, waypoint_id=None, reason=None):
        self.sequence += 1
        data = {'mission_id': self.mission_id, 'sequence': self.sequence, 'event': event}
        if waypoint_id is not None:
            data['waypoint_id'] = waypoint_id
        if reason is not None:
            data['reason'] = str(reason)
        self.publisher.publish(String(data=json.dumps(data, ensure_ascii=False)))

    def flush(self):
        # Prevent final completed/aborted events being lost at publisher shutdown.
        try:
            return self.publisher.wait_for_all_acked(Duration(seconds=0.5))
        except Exception:
            return False

class RouteSpeedPublisher:

    def __init__(self, nav, initial_limit):
        self.nav = nav
        self.limit = float(initial_limit)
        self.publisher = nav.create_publisher(
            SpeedLimit, '/speed_limit', 10
        )

        # Periodically reapply the current speed limit
        self.timer = nav.create_timer(0.1, self.publish)

    def set_limit(self, value):
        self.limit = float(value)
        self.publish()

    def publish(self):
        msg = SpeedLimit()
        msg.header.stamp = self.nav.get_clock().now().to_msg()
        msg.percentage = False
        msg.speed_limit = self.limit

        self.publisher.publish(msg)

def yaw_quaternion(yaw):
    return math.sin(yaw / 2), math.cos(yaw / 2)


def make_pose(nav, w, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = nav.get_clock().now().to_msg()
    pose.pose.position.x = w.x
    pose.pose.position.y = w.y
    pose.pose.orientation.z, pose.pose.orientation.w = yaw_quaternion(yaw)
    return pose


def wait_for_gate(nav, gate, max_wait=10.0):
    deadline = time.monotonic() + max_wait
    while time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=0.1)
        if gate.healthy():
            return
    require_healthy(gate, 'Route startup health check timed out')

def wait_stopped(nav, gate, motion, timeout=5.0):
    motion.reset_stop_confirmation()
    deadline = time.monotonic() + timeout

    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(nav, timeout_sec=0.05)

        require_healthy(gate, 'Health check failed while stopping')

        if motion.stopped():
            return

    raise RuntimeError('Robot stop could not be confirmed')


def verify_control_point(nav, tf_buffer, waypoint, tolerance):
    transform = tf_buffer.lookup_transform(
        'map', 'base_link', Time(),
    )

    stamp = Time.from_msg(transform.header.stamp)
    age = (nav.get_clock().now() - stamp).nanoseconds * 1e-9

    if not 0.0 <= age <= 0.5:
        raise RuntimeError('Robot transform is stale')

    pos = transform.transform.translation
    error = math.hypot(
        pos.x - waypoint.x, pos.y - waypoint.y,
    )

    if not math.isfinite(error) or error > tolerance:
        raise RuntimeError(
            f'{waypoint.id}: position error {error:.3f} m '
            f'exceeds tolerance {tolerance:.3f} m'
        )


def wait_at_waypoint(nav, gate, motion, tf_buffer, waypoint, tolerance):
    deadline = time.monotonic() + waypoint.wait_time_sec

    while rclpy.ok():
        rclpy.spin_once(nav, timeout_sec=0.05)

        require_healthy(gate, 'Health check failed during waypoint wait')

        if not motion.stopped():
            raise RuntimeError('Robot moved during waypoint wait')

        verify_control_point(nav, tf_buffer, waypoint, tolerance)

        if time.monotonic() >= deadline:
            return

    raise RuntimeError('ROS shutdown during waypoint wait')


def cancel_and_confirm(nav, timeout=5.0):
    """Request cancellation and wait for a terminal result within timeout."""
    goal_handle = getattr(nav, 'goal_handle', None)
    result_future = getattr(nav, 'result_future', None)

    if goal_handle is None or result_future is None:
        return False

    if result_future.done():
        nav.isTaskComplete()
        return True

    deadline = time.monotonic() + timeout
    try:
        cancel_future = goal_handle.cancel_goal_async()
    except Exception as error:
        nav.get_logger().error(f'Cancel request failed: {error}')
        return False

    accepted = False
    while rclpy.ok() and time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_once(nav, timeout_sec=min(0.1, remaining))

        if result_future.done():
            nav.isTaskComplete()
            return True

        if cancel_future.done():
            try:
                response = cancel_future.result()
                accepted = bool(response and response.goals_canceling)
            except Exception as error:
                nav.get_logger().error(f'Cancel response failed: {error}')
                return False
            break

    if not accepted:
        return False

    while rclpy.ok() and time.monotonic() < deadline:
        remaining = max(0.0, deadline - time.monotonic())
        rclpy.spin_once(nav, timeout_sec=min(0.1, remaining))

        if result_future.done():
            nav.isTaskComplete()
            return True

    return False


def abort_motion(nav, watchdog):
    hold_confirmed = True
    canceled = True
    if watchdog is not None:
        try:
            watchdog.close()
        except Exception as error:
            hold_confirmed = False
            nav.get_logger().error(
                f'Watchdog hold request failed: {error}'
            )
        finally:
            watchdog.stop_updates()
    future = getattr(nav, 'result_future', None)
    if future is not None and not future.done():
        try:
            canceled = cancel_and_confirm(nav)
        except Exception as error:
            canceled = False
            nav.get_logger().error(
                f'Navigation cancellation failed: {error}'
            )
    return hold_confirmed and canceled


def release_for_task(nav, gate, watchdog):
    require_healthy(gate, 'Health check failed before motion')
    if watchdog is not None:
        future = getattr(nav, 'result_future', None)
        if future is None:
            raise RuntimeError('Navigation task has no result future')
        if not future.done():
            watchdog.set_hold(False)


def wait_nav_task(nav, gate, timeout, watchdog=None):
    started = time.monotonic()
    while rclpy.ok():
        completed = nav.isTaskComplete()
        require_healthy(gate, 'Route health check failed during navigation')
        if completed:
            if watchdog is not None:
                watchdog.set_hold(True)
            return (
                'succeeded'
                if nav.getResult() == TaskResult.SUCCEEDED
                else 'failed'
            )
        if time.monotonic() - started > timeout:
            if watchdog is not None:
                watchdog.set_hold(True)
            if not cancel_and_confirm(nav):
                raise RuntimeError('Goal timeout: cancellation not confirmed')
            return 'timeout'
    raise RuntimeError('ROS shutdown during navigation')


def robot_yaw(tf_buffer):
    transform = tf_buffer.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=0.4))
    q = transform.transform.rotation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def normalized_angle(value):
    return (value + math.pi) % (2 * math.pi) - math.pi


def ensure_heading(nav, tf_buffer, gate, target_yaw, tolerance, timeout, watchdog=None):
    current = robot_yaw(tf_buffer)
    delta = normalized_angle(target_yaw - current)
    if abs(delta) <= tolerance:
        return True
    require_healthy(gate, 'Health check failed before yaw alignment')
    if not nav.spin(
            spin_dist=delta, time_allowance=math.ceil(timeout)):
        return False
    release_for_task(nav, gate, watchdog)
    if wait_nav_task(nav, gate, timeout, watchdog) != 'succeeded':
        return False
    return abs(
        normalized_angle(target_yaw - robot_yaw(tf_buffer))
    ) <= tolerance


def check_position_goal_checker(params_file):
    with Path(params_file).open('r', encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    params = data['controller_server']['ros__parameters']
    checker_names = params['goal_checker_plugins']
    if len(checker_names) != 1 or params[checker_names[0]]['plugin'] != 'nav2_controller::PositionGoalChecker':
        raise ValueError('Execution requires controller_server PositionGoalChecker for yaw-free waypoints')
    return params


def parse_args():
    parser = argparse.ArgumentParser(description='Safe map-based waypoint route execution')
    parser.add_argument('--route', type=Path, required=True, help='Saved YAML route')
    parser.add_argument('--map-yaml', type=Path, required=True, help='Static occupancy map YAML')
    parser.add_argument('--nav2-params', type=Path, required=True, help='Nav2 parameters YAML')
    parser.add_argument('--clearance', type=float, default=0.05, help='Additional robot footprint clearance [m]')
    parser.add_argument('--plan-preview', action='store_true', help='Ask Nav2 Planner for each segment, no motion')
    parser.add_argument('--execute', action='store_true', help='Actually send NavigateToPose and Spin goals')
    parser.add_argument('--route-control', action='store_true', help='Enable attributed route control with watchdog')
    parser.add_argument('--initial-speed-limit', type=float, default=0.25, help='Initial route speed limit [m/s]')
    parser.add_argument('--control-point-tolerance', type=float, default=0.25, help='Position tolerance for attributed waypoints [m]')
    parser.add_argument('--max-goal-time', type=float, default=120.0, help='Per-goal timeout [s]')
    parser.add_argument('--max-retries', type=int, default=1, help='Additional attempts for optional goals')
    parser.add_argument('--yaw-tolerance-deg', type=float, default=15.0, help='Required terminal yaw tolerance [deg]')
    parser.add_argument('--results-file', type=Path, help='Execution result log YAML')
    parser.add_argument('--localization-mode', choices=['slam', 'ekf'], default='slam', help='Localization method used by bringup')
    parser.add_argument('--ekf-3d', action='store_true', help='Require filtered IMU orientation for 3-D EKF')
    args = parser.parse_args()
    if args.max_goal_time <= 0 or not math.isfinite(args.max_goal_time) or args.max_retries < 0 or args.max_retries > 5:
        parser.error('Invalid goal timeout or retry count')
    if args.clearance < 0 or not math.isfinite(args.clearance):
        parser.error('Invalid clearance')
    if not 0.0 < args.yaw_tolerance_deg <= 180:
        parser.error('Invalid yaw tolerance')
    if args.route_control and not args.execute:
        parser.error('--route-control requires --execute')
    if args.plan_preview and args.execute:
        parser.error('--plan-preview and --execute are mutually exclusive')
    if (not math.isfinite(args.initial_speed_limit)
            or args.initial_speed_limit <= 0.0):
        parser.error('Invalid initial speed limit')
    if (not math.isfinite(args.control_point_tolerance)
            or args.control_point_tolerance <= 0.0):
        parser.error('Invalid control point tolerance')
    if args.ekf_3d and args.localization_mode != 'ekf':
        parser.error('--ekf-3d requires --localization-mode ekf')
    if args.execute and args.localization_mode == 'ekf' and not args.route_control:
        parser.error('EKF navigation requires --route-control')
        return args


def main():
    args = parse_args()
    map_file = args.map_yaml.expanduser().resolve()
    params_file = args.nav2_params.expanduser().resolve()
    route = load_route(args.route, str(map_file))
    route.validate(require_goals=True)
    grid = load_grid_map(map_file)
    footprint = load_footprint(params_file)
    filtered, skipped = filter_unsafe_route(route, grid, footprint, args.clearance)
    print(f'Route: {len(route.waypoints)} points; map-safe: {len(filtered.waypoints)}; skipped: {len(skipped)}')
    for entry in skipped:
        print(f'  SKIP {entry["id"]}: {entry["reason"]}')
    for i, wp in enumerate(filtered.waypoints):
        speed = 'inherit' if wp.speed_limit_mps is None else f'{wp.speed_limit_mps:.2f} m/s'
        print(f'  {i+1}. {wp.id}: ({wp.x:.2f},{wp.y:.2f}), '
              f'required={not wp.skippable}, yaw_required={wp.orientation_required}, '
              f'wait={wp.wait_time_sec:g} s, speed={speed}')
    if not args.plan_preview and not args.execute:
        print('OFFLINE PREVIEW ONLY. Add --plan-preview to query Nav2, --execute to move the robot.')
        return 0
    if args.execute:
        controller_params = check_position_goal_checker(params_file)
        has_attributes = any(
            w.wait_time_sec > 0.0 or w.speed_limit_mps is not None
            for w in route.waypoints
        )
        if has_attributes and not args.route_control:
            raise RuntimeError(
                'Attributed route requires --route-control'
            )
        if args.route_control:
            checker = controller_params['goal_checker_plugins'][0]
            xy_tolerance = float(
                controller_params[checker]['xy_goal_tolerance']
            )
            if not 0.0 < xy_tolerance <= args.control_point_tolerance:
                raise RuntimeError(
                    'Goal checker tolerance exceeds control point tolerance'
                )
            max_speed = float(
                controller_params['FollowPath']['vx_max']
            )
            if not 0.0 < args.initial_speed_limit <= max_speed:
                raise RuntimeError('Initial speed exceeds MPPI limit')
            for w in route.waypoints:
                if (w.speed_limit_mps is not None
                        and w.speed_limit_mps > max_speed):
                    raise RuntimeError(
                        f'{w.id}: speed exceeds MPPI limit'
                    )
    rclpy.init()
    nav = BasicNavigator(node_name='waypoint_route_runner')
    events = WaypointEventPublisher(nav) if args.execute else None
    gate = UrgHealthGate(nav) if args.execute else None
    tf_buffer = Buffer() if args.execute else None
    tf_listener = TransformListener(tf_buffer, nav, spin_thread=False) if args.execute else None
    watchdog = None
    speed_publisher = None
    motion = None
    results = [{'id': s['id'], 'status': 'skipped', 'reason': s['reason']} for s in skipped]
    preflight_skips = {entry['id']: entry for entry in results}
    try:
        if args.execute:
            nav.waitUntilNav2Active(localizer='map_server')
            wait_for_gate(nav, gate)
        else:
            nav.waitUntilNav2Active(navigator='planner_server', localizer='map_server')
        # Preflight all map-safe segments. If an optional segment is not plannable,
        # remove its target and plan the next segment from the last accepted pose.
        accepted = []
        previous = None
        start_index = 0
        if not args.execute:
            first = filtered.waypoints[0]
            previous = make_pose(nav, first, route_heading(filtered, 0))
            accepted.append((first, previous))
            start_index = 1
        for i in range(start_index, len(filtered.waypoints)):
            w = filtered.waypoints[i]
            pose = make_pose(nav, w, route_heading(filtered, i))
            print(f'[{i+1}/{len(filtered.waypoints)}] Planning to {w.id}...', flush=True)
            planned = nav.getPath(previous if previous is not None else pose, pose, use_start=(previous is not None))
            if planned is None or not planned.poses:
                if not w.skippable:
                    raise RuntimeError(f'No valid plan to required waypoint {w.id}; mission aborted')
                entry = {'id': w.id, 'status': 'skipped', 'reason': 'preflight_planner'}
                results.append(entry)
                preflight_skips[w.id] = entry
                print(f'SKIP {w.id}: planner returned no path')
                continue
            accepted.append((w, pose))
            previous = pose
        if not accepted or accepted[-1][0].skippable:
            raise RuntimeError('No reachable required final goal')
        print(f'Preflight complete: {len(accepted)} reachable waypoints')
        if not args.execute:
            print('PLAN PREVIEW ONLY: no movement commands sent')
            return 0
        if args.route_control:
            watchdog = WatchdogClient(
                nav, initial_limit=args.initial_speed_limit,
            )
            watchdog.start()
            localization_gate = LocalizationHealthGate(nav)
            motion = RobotMotionMonitor(nav)
            gate = RouteHealthGate(
                gate, localization_gate, motion, watchdog,
            )
            input_gates = localization_input_gates(
                nav,
                args.localization_mode,
                require_filtered_imu=args.ekf_3d,
            )
            gate = RouteHealthGate(
                gate, localization_gate, motion, watchdog,
                extra_gates=input_gates,
            )
            speed_publisher = RouteSpeedPublisher(
                nav, args.initial_speed_limit,
            )
            speed_publisher.publish()
            wait_for_gate(nav, gate)
            wait_stopped(nav, gate, motion)

        events.send('started')
        accepted_by_id = {w.id: (w, pose) for w, pose in accepted}
        for original in route.waypoints:
            prior_skip = preflight_skips.get(original.id)
            if prior_skip is not None:
                events.send('skipped', original.id, prior_skip['reason'])
                continue
            w, pose = accepted_by_id[original.id]
            require_healthy(gate, 'Route health check failed before goal')
            # Update pose stamp and re-plan from live pose before each goal.
            pose.header.stamp = nav.get_clock().now().to_msg()
            planned = nav.getPath(pose, pose, use_start=False)
            if planned is None or not planned.poses:
                if w.skippable:
                    results.append({'id': w.id, 'status': 'skipped', 'reason': 'live_planner'})
                    events.send('skipped', w.id, 'live_planner')
                    continue
                raise RuntimeError(f'Live path unavailable to required {w.id}')
            state = 'failed'
            for attempt in range(args.max_retries + 1):
                require_healthy(gate, 'Route health check failed before goal')
                if attempt > 0:
                    events.send('retrying', w.id)
                pose.header.stamp = nav.get_clock().now().to_msg()
                if not nav.goToPose(pose):
                    state = 'rejected'
                    break
                release_for_task(nav, gate, watchdog)
                state = wait_nav_task(nav, gate, args.max_goal_time, watchdog)
                print(f'{w.id}: attempt {attempt+1} -> {state}')
                if state == 'succeeded':
                    break
            if state == 'succeeded' and w.orientation_required:
                if watchdog is not None:
                    watchdog.set_hold(True)
                    wait_stopped(nav, gate, motion)
                    
                try:
                    rotated = ensure_heading(
                        nav, tf_buffer, gate, w.yaw,
                        math.radians(args.yaw_tolerance_deg),
                        30.0, watchdog,
                    )
                except TransformException as error:
                    rotated = False
                    print(f'Yaw verification unavailable: {error}', file=sys.stderr)
                if not rotated:
                    state = 'yaw_failed'
            if state == 'succeeded':
                control_point = (
                    w.wait_time_sec > 0.0
                    or w.speed_limit_mps is not None
                )
                if control_point:
                    watchdog.set_hold(True)
                    wait_stopped(nav, gate, motion)
                    verify_control_point(
                        nav, tf_buffer, w,
                        args.control_point_tolerance,
                    )
                results.append({'id': w.id, 'status': 'reached'})
                events.send('reached', w.id)
                # Wait at the waypoint
                if w.wait_time_sec > 0.0:
                    events.send('waiting', w.id)
                    wait_at_waypoint(
                        nav, gate, motion, tf_buffer, w,
                        args.control_point_tolerance,
                    )
                    events.send('wait_completed', w.id)
                # Set the speed limit for the following segment
                if w.speed_limit_mps is not None:
                    watchdog.set_speed_limit(w.speed_limit_mps)
                    speed_publisher.set_limit(w.speed_limit_mps)
                    events.send(
                        'speed_changed', w.id,
                        reason=str(w.speed_limit_mps),
                    )
            elif w.skippable:
                results.append({'id': w.id, 'status': 'skipped', 'reason': state})
                events.send('skipped', w.id, state)
            else:
                raise RuntimeError(f'Required waypoint {w.id} failed: {state}')
        if watchdog is not None:
            watchdog.set_hold(True)
            wait_stopped(nav, gate, motion)
            watchdog.close()
        events.send('completed')
        print('Mission complete; required goal reached.')
        return 0
    except KeyboardInterrupt:
        print('Interrupted; stopping navigation', file=sys.stderr)
        stopped = not args.execute or abort_motion(nav, watchdog)
        if events is not None:
            events.send(
                'aborted' if stopped else 'safety_warning',
                reason='keyboard_interrupt',
            )
        if not stopped:
            print(
                'WARNING: stop hold or cancellation not confirmed',
                file=sys.stderr,
            )
        return 130
    except Exception as error:
        stopped = not args.execute or abort_motion(nav, watchdog)
        if events is not None:
            events.send(
                'aborted' if stopped else 'safety_warning',
                reason=error,
            )
        print(f'ABORTED: {error}', file=sys.stderr)
        results.append({
            'id': 'mission',
            'status': 'aborted',
            'reason': str(error),
        })
        return 1
    finally:
        if watchdog is not None:
            watchdog.stop_updates()
        if events is not None:
            events.flush()
        if args.execute:
            log = args.results_file or args.route.expanduser().with_suffix('.results.yaml')
            report = {'timestamp_utc': datetime.now(timezone.utc).isoformat(),
                      'route': str(args.route), 'results': results}
            log = Path(log).expanduser().resolve()
            # Result file must never overwrite route or source map.
            if log not in {args.route.expanduser().resolve(), map_file}:
                log.parent.mkdir(parents=True, exist_ok=True)
                with log.open('w', encoding='utf-8') as file:
                    yaml.safe_dump(report, file, sort_keys=False, allow_unicode=True)
        nav.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
