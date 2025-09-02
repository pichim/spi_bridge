from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("spi_bridge"), "config", "params.yaml"])
    return LaunchDescription(
        [
            # Optional arg if you want to swap param files later
            DeclareLaunchArgument("params_file", default_value=params),
            Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="teleop",
                output="screen",
                emulate_tty=True,  # (1) gives teleop a pseudo-TTY so keypresses work under launch
                remappings=[("cmd_vel", "/cmd_vel")],
                parameters=[],  # teleop has no params we need to set here
            ),
            Node(
                package="spi_bridge",
                executable="spi_bridge_node",
                name="spi_bridge",
                output="screen",
                parameters=[params],  # (4) load parameters from YAML
            ),
        ]
    )
