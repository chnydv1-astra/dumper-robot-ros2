from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    slam_config = PathJoinSubstitution([
        FindPackageShare("dumper_description"),
        "config",
        "slam.yaml"
    ])

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {
                "use_sim_time": True,
                "minimum_laser_range": 0.2,
                "max_laser_range": 30.0,
                "map_frame": "map",
                "odom_frame": "dumper1/odom",
                "base_frame": "dumper1/base_link",
                "scan_topic": "/dumper1/scan",
            }
        ]
    )

    return LaunchDescription([
        slam_node
    ])