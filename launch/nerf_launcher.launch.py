from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="",
                description="Namespace for nerf launcher node",
            ),
            Node(
                package="robot_nerf_launcher",
                executable="nerf_launcher_node",
                namespace=namespace,
                name="nerf_launcher",
                output="screen",
                parameters=[
                    {"pan_joint_name": "pan_servo_joint"},
                    {"tilt_joint_name": "tilt_servo_joint"},
                    {"flywheel_left_name": "flywheel_left"},
                    {"flywheel_right_name": "flywheel_right"},
                ],
            ),
        ]
    )

