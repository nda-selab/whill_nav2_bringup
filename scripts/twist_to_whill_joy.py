#!/usr/bin/env python3

from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


class TwistToWhillJoy(Node):
    """geometry_msgs/TwistをWHILL仮想ジョイスティックへ変換する。"""

    def __init__(self) -> None:
        super().__init__("twist_to_whill_joy")

        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter(
            "output_topic",
            "/whill/controller/joy",
        )
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("command_timeout", 0.25)

        # Joy軸が1.0となる基準速度。
        # 実機試験で調整する。
        self.declare_parameter("max_linear_speed", 0.60)
        self.declare_parameter("max_angular_speed", 1.20)

        self.declare_parameter("linear_scale", 1.0)
        self.declare_parameter("angular_scale", 1.0)

        # 正負が逆の場合は-1.0へ変更する。
        self.declare_parameter("angular_sign", 1.0)

        self.declare_parameter("deadband", 0.02)

        self.input_topic = (
            self.get_parameter("input_topic")
            .get_parameter_value()
            .string_value
        )
        self.output_topic = (
            self.get_parameter("output_topic")
            .get_parameter_value()
            .string_value
        )

        self.publish_rate = (
            self.get_parameter("publish_rate")
            .get_parameter_value()
            .double_value
        )
        self.command_timeout = (
            self.get_parameter("command_timeout")
            .get_parameter_value()
            .double_value
        )
        self.max_linear_speed = (
            self.get_parameter("max_linear_speed")
            .get_parameter_value()
            .double_value
        )
        self.max_angular_speed = (
            self.get_parameter("max_angular_speed")
            .get_parameter_value()
            .double_value
        )
        self.linear_scale = (
            self.get_parameter("linear_scale")
            .get_parameter_value()
            .double_value
        )
        self.angular_scale = (
            self.get_parameter("angular_scale")
            .get_parameter_value()
            .double_value
        )
        self.angular_sign = (
            self.get_parameter("angular_sign")
            .get_parameter_value()
            .double_value
        )
        self.deadband = (
            self.get_parameter("deadband")
            .get_parameter_value()
            .double_value
        )

        if self.publish_rate <= 0.0:
            raise ValueError("publish_rateは0より大きくしてください")

        if self.command_timeout <= 0.0:
            raise ValueError(
                "command_timeoutは0より大きくしてください"
            )

        if self.max_linear_speed <= 0.0:
            raise ValueError(
                "max_linear_speedは0より大きくしてください"
            )

        if self.max_angular_speed <= 0.0:
            raise ValueError(
                "max_angular_speedは0より大きくしてください"
            )

        self.publisher = self.create_publisher(
            Joy,
            self.output_topic,
            10,
        )

        self.subscription = self.create_subscription(
            Twist,
            self.input_topic,
            self.on_twist,
            10,
        )

        self.latest_twist = Twist()
        self.last_command_time: float | None = None

        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.on_timer,
        )

        self.get_logger().info(
            f"Twist→WHILL Joy bridge: "
            f"{self.input_topic} → {self.output_topic}"
        )
        self.get_logger().info(
            f"max_linear_speed={self.max_linear_speed:.3f} m/s, "
            f"max_angular_speed={self.max_angular_speed:.3f} rad/s"
        )

    def on_twist(self, message: Twist) -> None:
        values = (
            message.linear.x,
            message.angular.z,
        )

        if not all(math.isfinite(value) for value in values):
            self.get_logger().warning(
                "非有限値を含むTwistを無視します"
            )
            return

        self.latest_twist = message
        self.last_command_time = time.monotonic()

    def apply_deadband(self, value: float) -> float:
        if abs(value) < self.deadband:
            return 0.0
        return value

    def publish_neutral(self) -> None:
        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()
        message.axes = [0.0, 0.0]
        message.buttons = []
        self.publisher.publish(message)

    def on_timer(self) -> None:
        now = time.monotonic()

        if (
            self.last_command_time is None
            or now - self.last_command_time
            > self.command_timeout
        ):
            self.publish_neutral()
            return

        forward = (
            self.latest_twist.linear.x
            / self.max_linear_speed
            * self.linear_scale
        )

        turn = (
            self.latest_twist.angular.z
            / self.max_angular_speed
            * self.angular_scale
            * self.angular_sign
        )

        forward = self.apply_deadband(
            clamp(forward, -1.0, 1.0)
        )
        turn = self.apply_deadband(
            clamp(turn, -1.0, 1.0)
        )

        message = Joy()
        message.header.stamp = self.get_clock().now().to_msg()

        # WHILLドライバでは、
        # axes[1]が前後、axes[0]が左右として使用される。
        message.axes = [
            float(turn),
            float(forward),
        ]
        message.buttons = []

        self.publisher.publish(message)


def main() -> None:
    rclpy.init()
    node = TwistToWhillJoy()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 終了前に複数回、中立指令を送る。
        for _ in range(3):
            node.publish_neutral()
            time.sleep(0.05)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
