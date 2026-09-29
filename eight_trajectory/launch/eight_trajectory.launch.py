from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Coach支持：入口和环境参数。默认保留课程/odom接口。
    odom_topic = LaunchConfiguration("odom_topic")
    return LaunchDescription([
        DeclareLaunchArgument("odom_topic", default_value="/odom"),
        # TODO CP16-C061：添加eight_trajectory与kinematic_model两个Node。
        # 每个Node配置package、executable、output="screen"。
        # eight_trajectory还需remappings=[("/odom", odom_topic)]。
        # 只启动这两个节点；两个Node之间用逗号分隔。
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