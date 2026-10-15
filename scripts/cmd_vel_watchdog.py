#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelWatchdog(Node):

    def __init__(self):
        super().__init__('cmd_vel_watchdog')

        self.declare_parameter('input_topic', '/cmd_vel')
        self.declare_parameter('output_topic', '/whill/controller/cmd_vel')
        self.declare_parameter('command_timeout_sec', 0.4)
        self.declare_parameter('publish_frequency', 20.0)
        self.declare_parameter('max_forward_speed', 0.25)
        self.declare_parameter('max_reverse_speed', 0.15)
        self.declare_parameter('max_angular_speed', 0.5)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.timeout = float(
            self.get_parameter('command_timeout_sec').value
        )
        frequency = float(
            self.get_parameter('publish_frequency').value
        )

        self.max_forward = float(
            self.get_parameter('max_forward_speed').value
        )
        self.max_reverse = float(
            self.get_parameter('max_reverse_speed').value
        )
        self.max_angular = float(
            self.get_parameter('max_angular_speed').value
        )

        if input_topic == output_topic:
            raise ValueError('Input and output topics must differ')

        values = [
            self.timeout,
            frequency,
            self.max_forward,
            self.max_reverse,
            self.max_angular,
        ]
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('Invalid watchdog configuration')

        self.last_received = None
        self.last_command = None

        self.publisher = self.create_publisher(
            Twist, output_topic, 10
        )
        self.subscription = self.create_subscription(
            Twist, input_topic, self.on_command, 10
        )
        self.timer = self.create_timer(
            1.0 / frequency, self.on_timer
        )

    def on_command(self, msg):
        values = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
            msg.angular.z,
        ]

        if not all(math.isfinite(v) for v in values):
            self.last_command = None
            self.last_received = None
            self.get_logger().error('Invalid velocity command')
            return

        if any(abs(v) > 1e-6 for v in [
            msg.linear.y,
            msg.linear.z,
            msg.angular.x,
            msg.angular.y,
        ]):
            self.last_command = None
            self.last_received = None
            self.get_logger().error('Unsupported velocity command')
            return

        command = Twist()

        command.linear.x = min(
            self.max_forward,
            max(-self.max_reverse, msg.linear.x),
        )
        command.angular.z = min(
            self.max_angular,
            max(-self.max_angular, msg.angular.z),
        )

        self.last_command = command
        self.last_received = time.monotonic()

    def on_timer(self):
        now = time.monotonic()

        valid = (
            self.last_command is not None
            and self.last_received is not None
            and now - self.last_received <= self.timeout
        )

        if valid:
            self.publisher.publish(self.last_command)
        else:
            self.publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = CmdVelWatchdog()
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()