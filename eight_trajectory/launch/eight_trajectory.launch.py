from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    odom_topic = LaunchConfiguration("odom_topic")
    trajectory_node = Node(
        package="eight_trajectory",
        executable="eight_trajectory",
        output="screen",
        remappings=[("/odom", odom_topic)],
    )
    converter_node = Node(
        package="kinematic_model",
        executable="kinematic_model",
        output="screen",
    )
    completion_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=trajectory_node,
            on_exit=[EmitEvent(event=Shutdown(reason="Trajectory process exited"))],
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
        # Register before starting the process so an early exit is also handled.
        completion_handler,
        converter_node,
        trajectory_node,
    ])
