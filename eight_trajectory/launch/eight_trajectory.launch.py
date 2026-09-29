from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    odom_topic = LaunchConfiguration("odom_topic")
    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        Node(
            package="eight_trajectory",
            executable="eight_trajectory",
            output="screen",
            remappings=[("/odom", odom_topic)]
        ),

        Node(
            package="kinematic_model",
            executable="kinematic_model",
            output="screen"
        )
    ])
