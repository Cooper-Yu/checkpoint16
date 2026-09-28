from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Coach支持：导入与入口；C038由学习者配置两个节点。
    return LaunchDescription([
        # TODO CP16-C038：添加轮速发布节点与运动学节点的Node配置。
        # 每个Node指定package、executable和output="screen"。
        Node(
            package="wheel_velocities_publisher",
            executable="wheel_velocities_publisher",
            output="screen"
        ),

        Node(
            package="kinematic_model",
            executable="kinematic_model",
            output="screen"
        )

    ])
