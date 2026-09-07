"""ROS adapter for the final velocity gate; uses a steady watchdog clock."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.clock import Clock, ClockType
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from turtlebot3_multimodal.executor import Velocity
from turtlebot3_multimodal.velocity_gate import VelocityGate


class VelocityGateNode(Node):
    def __init__(self):
        super().__init__("velocity_gate")
        self.gate = VelocityGate()
        self.publisher = self.create_publisher(Twist, "/cmd_vel_safe", 1)
        self.create_subscription(Twist, "/cmd_vel", self.nav, 1)
        self.create_subscription(Twist, "/turtlebot3/manual_cmd_vel", self.manual, 1)
        self.create_service(Trigger, "/turtlebot3/emergency_stop", self.stop)
        self.create_service(Trigger, "/turtlebot3/reset_stop", self.reset)
        self.create_service(SetBool, "/turtlebot3/select_manual", self.select)
        self.create_timer(0.02, self.publish, clock=Clock(clock_type=ClockType.STEADY_TIME))

    def nav(self, message):
        self.receive("nav", message)

    def manual(self, message):
        self.receive("manual", message)

    def receive(self, source, message):
        self.gate.receive(source, Velocity(message.linear.x, message.angular.z), time.monotonic())

    def publish(self):
        velocity = self.gate.tick(time.monotonic())
        message = Twist()
        message.linear.x, message.angular.z = velocity.linear_x, velocity.angular_z
        self.publisher.publish(message)

    def stop(self, request, response):
        self.gate.stop()
        self.publish()
        response.success, response.message = True, "final velocity stop latched"
        return response

    def reset(self, request, response):
        self.gate.reset()
        response.success, response.message = True, "reset; fresh selected-source input required"
        return response

    def select(self, request, response):
        self.gate.select("manual" if request.data else "nav")
        self.publish()
        response.success, response.message = True, self.gate.source
        return response


def main(args=None):
    rclpy.init(args=args)
    node = VelocityGateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.gate.stop()
        if rclpy.ok():
            node.publish()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
