"""ROS 2 Humble adapter for the safe command core."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from turtlebot3_multimodal.commands import (
    CommandValidationError,
    SafetyLimits,
    parse_command_payload,
)
from turtlebot3_multimodal.executor import MotionExecutor, Velocity


class SafeCommandNode(Node):
    def __init__(self) -> None:
        super().__init__("safe_multimodal_controller")
        self.declare_parameter("max_distance_m", 2.0)
        self.declare_parameter("max_angle_deg", 360.0)
        self.declare_parameter("max_linear_mps", 0.22)
        self.declare_parameter("max_angular_rps", 1.5)
        self.declare_parameter("watchdog_timeout_s", 0.5)
        limits = SafetyLimits(
            max_distance_m=float(self.get_parameter("max_distance_m").value),
            max_angle_deg=float(self.get_parameter("max_angle_deg").value),
            max_linear_mps=float(self.get_parameter("max_linear_mps").value),
            max_angular_rps=float(self.get_parameter("max_angular_rps").value),
            watchdog_timeout_s=float(self.get_parameter("watchdog_timeout_s").value),
        )
        self._limits = limits
        self._executor = MotionExecutor(limits)
        self._publisher = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(String, "/turtlebot3/command", self._on_command, 10)
        self.create_service(Trigger, "/turtlebot3/emergency_stop", self._on_stop)
        self.create_service(Trigger, "/turtlebot3/reset_stop", self._on_reset)
        self.create_timer(0.05, self._on_timer)

    def _publish(self, velocity: Velocity) -> None:
        message = Twist()
        message.linear.x = velocity.linear_x
        message.angular.z = velocity.angular_z
        self._publisher.publish(message)

    def _on_command(self, message: String) -> None:
        try:
            plan = parse_command_payload(message.data, self._limits)
            self._executor.submit(plan)
            self.get_logger().info(
                f"accepted {len(plan.commands)} command(s) from {plan.source}"
            )
        except (CommandValidationError, RuntimeError) as error:
            self._publish(Velocity())
            self.get_logger().warning(f"rejected command: {error}")

    def _on_timer(self) -> None:
        now_s = self.get_clock().now().nanoseconds / 1e9
        self._publish(self._executor.tick(now_s))

    def _on_stop(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._publish(self._executor.emergency_stop())
        response.success = True
        response.message = "emergency stop latched"
        return response

    def _on_reset(self, request: Trigger.Request, response: Trigger.Response) -> Trigger.Response:
        del request
        self._executor.reset_emergency_stop()
        response.success = True
        response.message = "emergency stop reset"
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SafeCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._publish(Velocity())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
