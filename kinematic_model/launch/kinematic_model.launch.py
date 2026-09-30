from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    wheel_publisher_node = Node(
        package="wheel_velocities_publisher",
        executable="wheel_velocities_publisher",
        output="screen",
    )
    converter_node = Node(
        package="kinematic_model",
        executable="kinematic_model",
        output="screen",
    )
    completion_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=wheel_publisher_node,
            on_exit=[EmitEvent(event=Shutdown(reason="Wheel publisher process exited"))],
        )
    )
    return LaunchDescription([
        completion_handler,
        converter_node,
        wheel_publisher_node,
    ])
