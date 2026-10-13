#!/usr/bin/env python3
"""Convert IMU-based odometry to base_link odometry for WHILL/Nav2.

Input:  /Odometry (pose: odom -> IMU, twist: IMU coordinates)
Output: /odom_nav (pose: odom -> base_link, twist: base_link coordinates)
        /tf odom -> base_link
Uses the existing static TF chain from base_link to the IMU frame.
"""

import copy
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformException


def quaternion_product(a, b):
    """Hamilton product, quaternions in ROS order (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    ], dtype=float)


def rotation_matrix(q):
    x, y, z, w = q
    return np.array([
        [1 - 2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1 - 2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1 - 2*(x*x+y*y)],
    ], dtype=float)


def skew(v):
    x, y, z = v
    return np.array([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)


def normalized(q):
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n < 1e-9:
        raise ValueError('Invalid quaternion')
    return q / n


def as_quaternion(msg):
    return normalized(np.array([msg.x, msg.y, msg.z, msg.w], dtype=float))


class OdometryToBaseLink(Node):
    def __init__(self):
        super().__init__('odom_to_base_link')
        self.declare_parameter('input_topic', '/Odometry')
        self.declare_parameter('output_topic', '/odom_nav')
        self.declare_parameter('base_frame', 'base_link')
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.base_frame = self.get_parameter('base_frame').value
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.broadcaster = TransformBroadcaster(self)
        self.publisher = self.create_publisher(Odometry, self.output_topic, 20)
        self.subscriber = self.create_subscription(Odometry, self.input_topic, self.on_odometry, 20)
        self.missing_tf_warned = False
        self.missing_cov_warned = False

    def on_odometry(self, msg):
        imu_frame = msg.child_frame_id
        if not imu_frame or not msg.header.frame_id or imu_frame == self.base_frame:
            self.get_logger().error(
                'Expected nonempty odom frame and IMU child_frame_id distinct from base_link')
            return

        # lookupTransform(target=IMU, source=base_link) gives T_imu_base.
        try:
            t = self.buffer.lookup_transform(imu_frame, self.base_frame, Time())
        except TransformException as error:
            if not self.missing_tf_warned:
                self.get_logger().warn(f'Waiting for static TF {imu_frame} <- {self.base_frame}: {error}')
                self.missing_tf_warned = True
            return
        self.missing_tf_warned = False

        try:
            q_odom_imu = as_quaternion(msg.pose.pose.orientation)
            q_imu_base = as_quaternion(t.transform.rotation)
        except ValueError as error:
            self.get_logger().error(str(error))
            return

        p_odom_imu = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=float)
        p_imu_base = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z], dtype=float)
        v_imu = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z], dtype=float)
        w_imu = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z], dtype=float)
        if not all(np.all(np.isfinite(v)) for v in (p_odom_imu, p_imu_base, v_imu, w_imu)):
            self.get_logger().error('Odometry or static TF contains NaN/Inf')
            return

        R_odom_imu = rotation_matrix(q_odom_imu)
        R_imu_base = rotation_matrix(q_imu_base)
        R_base_imu = R_imu_base.T
        lever_arm_odom = R_odom_imu @ p_imu_base

        p_odom_base = p_odom_imu + lever_arm_odom
        q_odom_base = normalized(quaternion_product(q_odom_imu, q_imu_base))
        v_base = R_base_imu @ (v_imu + np.cross(w_imu, p_imu_base))
        w_base = R_base_imu @ w_imu

        out = copy.deepcopy(msg)
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x, out.pose.pose.position.y, out.pose.pose.position.z = map(float, p_odom_base)
        (out.pose.pose.orientation.x, out.pose.pose.orientation.y,
         out.pose.pose.orientation.z, out.pose.pose.orientation.w) = map(float, q_odom_base)
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = map(float, v_base)
        out.twist.twist.angular.x, out.twist.twist.angular.y, out.twist.twist.angular.z = map(float, w_base)

        # First-order pose covariance correction for displacement of the origin.
        # Assumes orientation error axes are represented in the odom frame.
        J_pose = np.eye(6)
        J_pose[:3, 3:] = -skew(lever_arm_odom)
        C_pose = np.array(msg.pose.covariance, dtype=float).reshape(6, 6)
        out.pose.covariance = (J_pose @ C_pose @ J_pose.T).reshape(36).tolist()

        C_twist = np.array(msg.twist.covariance, dtype=float).reshape(6, 6)
        if not np.any(C_twist):
            # FAST-LIO2 currently provides no twist covariance. Provisional only.
            out.twist.covariance = np.diag([
                0.05**2, 0.05**2, 0.10**2,
                0.10**2, 0.10**2, 0.05**2]).reshape(36).tolist()
            if not self.missing_cov_warned:
                self.get_logger().warn('Input twist covariance is zero; publishing provisional diagonal covariance')
                self.missing_cov_warned = True
        else:
            J_twist = np.zeros((6, 6))
            J_twist[:3, :3] = R_base_imu
            J_twist[:3, 3:] = -R_base_imu @ skew(p_imu_base)
            J_twist[3:, 3:] = R_base_imu
            out.twist.covariance = (J_twist @ C_twist @ J_twist.T).reshape(36).tolist()

        transform = TransformStamped()
        transform.header = copy.deepcopy(msg.header)
        transform.child_frame_id = self.base_frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = map(float, p_odom_base)
        (transform.transform.rotation.x, transform.transform.rotation.y,
         transform.transform.rotation.z, transform.transform.rotation.w) = map(float, q_odom_base)

        self.publisher.publish(out)
        self.broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = OdometryToBaseLink()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()