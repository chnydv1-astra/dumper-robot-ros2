from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    ekf_config = PathJoinSubstitution([
        FindPackageShare("dumper_description"),
        "config",
        "ekf.yaml"
    ])

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        namespace="dumper1",
        output="screen",
        parameters=[
            ekf_config,
            {
                "use_sim_time": True
            }
        ]
    )

    return LaunchDescription([
        ekf_node
    ])