"""Headless TurtleBot3 world with online SLAM, Nav2, and an evaluation goal."""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter

from launch import LaunchDescription
from turtlebot3_multimodal.guarded_model import write_guarded_model


def generate_launch_description() -> LaunchDescription:
    model_directory = tempfile.TemporaryDirectory(prefix="turtlebot3-guarded-")
    robot_sdf = write_guarded_model(
        os.path.join(get_package_share_directory("nav2_bringup"), "worlds", "waffle.model"),
        os.path.join(model_directory.name, "waffle.model"),
    )
    nav2_launch = os.path.join(
        get_package_share_directory("nav2_bringup"), "launch", "tb3_simulation_launch.py"
    )
    output_dir = LaunchConfiguration("output_dir")
    evaluator = Node(
        package="turtlebot3_multimodal",
        executable="navigation_evaluator",
        output="screen",
        arguments=[
            "--goal-x",
            LaunchConfiguration("goal_x"),
            "--goal-y",
            LaunchConfiguration("goal_y"),
            "--timeout",
            LaunchConfiguration("timeout"),
            "--output-dir",
            output_dir,
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("goal_x", default_value="-1.2"),
            DeclareLaunchArgument("goal_y", default_value="-0.5"),
            DeclareLaunchArgument("timeout", default_value="90.0"),
            DeclareLaunchArgument("output_dir", default_value="/tmp/turtlebot3-navigation"),
            SetParameter(name="use_sim_time", value=True),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "headless": "True",
                    "use_rviz": "False",
                    "slam": "True",
                    "use_sim_time": "True",
                    "use_composition": "False",
                    "robot_sdf": robot_sdf,
                }.items(),
            ),
            Node(package="turtlebot3_multimodal", executable="velocity_gate", output="screen"),
            Node(
                package="turtlebot3_multimodal", executable="safe_controller", output="screen",
                remappings=[
                    ("/cmd_vel", "/turtlebot3/manual_cmd_vel"),
                    ("/turtlebot3/emergency_stop", "/turtlebot3/manual/emergency_stop"),
                    ("/turtlebot3/reset_stop", "/turtlebot3/manual/reset_stop"),
                ],
            ),
            evaluator,
            RegisterEventHandler(OnShutdown(
                on_shutdown=lambda event, context: model_directory.cleanup()
            )),
            RegisterEventHandler(
                OnProcessExit(
                    target_action=evaluator,
                    on_exit=[EmitEvent(event=Shutdown(reason="navigation evaluation finished"))],
                )
            ),
        ]
    )
