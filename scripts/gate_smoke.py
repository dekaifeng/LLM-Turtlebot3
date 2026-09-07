"""Exercise final-gate topics and services in a real ROS graph, without a robot."""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_srvs.srv import SetBool, Trigger

from turtlebot3_multimodal.gate_node import VelocityGateNode


def main():
    rclpy.init()
    gate, client = VelocityGateNode(), Node("velocity_gate_probe")
    executor = SingleThreadedExecutor()
    executor.add_node(gate)
    executor.add_node(client)
    received = []
    client.create_subscription(Twist, "/cmd_vel_safe", received.append, 10)
    nav = client.create_publisher(Twist, "/cmd_vel", 1)
    manual = client.create_publisher(Twist, "/turtlebot3/manual_cmd_vel", 1)

    def spin(duration, publisher=None, velocity=0.1):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            if publisher is not None:
                message = Twist()
                message.linear.x = velocity
                publisher.publish(message)
            executor.spin_once(timeout_sec=0.01)

    def service(name, kind, request):
        connection = client.create_client(kind, name)
        assert connection.wait_for_service(timeout_sec=5), name
        future = connection.call_async(request)
        executor.spin_until_future_complete(future, timeout_sec=5)
        assert future.done() and future.result().success, name
        client.destroy_client(connection)
        spin(0.1)
        received.clear()

    try:
        spin(1.0, nav)
        assert any(message.linear.x > 0 for message in received), "nav did not reach final output"
        service("/turtlebot3/emergency_stop", Trigger, Trigger.Request())
        spin(0.2, nav)
        assert received and all(message.linear.x == 0 for message in received), "stop bypassed"
        service("/turtlebot3/reset_stop", Trigger, Trigger.Request())
        request = SetBool.Request()
        request.data = True
        service("/turtlebot3/select_manual", SetBool, request)
        spin(0.2, nav)
        assert received and all(message.linear.x == 0 for message in received), "source conflict"
        spin(0.2, manual, -0.1)
        assert any(message.linear.x < 0 for message in received), "manual source did not move"
        spin(0.7)
        received.clear()
        spin(0.2, manual, -0.1)
        assert received and all(message.linear.x == 0 for message in received), "watchdog not latched"
        print("gate ROS smoke: navigation, emergency stop, source isolation, watchdog passed")
    finally:
        executor.shutdown()
        gate.destroy_node()
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
