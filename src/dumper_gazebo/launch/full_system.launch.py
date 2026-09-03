from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():

    # ================= GAZEBO =================

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("dumper_gazebo"),
                "launch",
                "sim.launch.py"
            ])
        )
    )

    # ================= EKF =================

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("dumper_description"),
                "launch",
                "ekf.launch.py"
            ])
        )
    )

    # ================= SLAM =================

    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("dumper_description"),
                "launch",
                "slam.launch.py"
            ])
        )
    )

    v2v_dumper1 = Node(
        package="dumper_v2v",
        executable="v2v_node",
        namespace="dumper1",
        parameters=[{
            "odom_topic": "/dumper1/odometry/filtered",
            "peer_topic": "/dumper2/v2v/state",
            "world_offset_x": 0.0,
            "world_offset_y": 0.0
        }],
        output="screen"
    )

    v2v_dumper2 = Node(
        package="dumper_v2v",
        executable="v2v_node",
        namespace="dumper2",
        parameters=[{
            "odom_topic": "/dumper2/odom",
            "peer_topic": "/dumper1/v2v/state",
            "world_offset_x": 0.0,
            "world_offset_y": 35.0,
            "yaw_offset": -1.5708
        }],
        output="screen"
    )
    

    # ================= STARTUP SEQUENCE =================

    # Give Gazebo enough time to start, spawn robots,
    # and start the bridge before sensor-dependent nodes.

    delayed_ekf = TimerAction(
        period=7.0,
        actions=[ekf_launch]
    )

    delayed_slam = TimerAction(
        period=9.0,
        actions=[slam_launch]
    )

    delayed_v2v = TimerAction(
        period=11.0,
        actions=[
            v2v_dumper1,
            v2v_dumper2
        ]
    )

    

    return LaunchDescription([

        # Gazebo + robots + bridge + RViz
        gazebo_launch,

        # EKF
        delayed_ekf,

        # SLAM
        delayed_slam,

        # V2V
        delayed_v2v,

        # Collision predictor
       
    ])
