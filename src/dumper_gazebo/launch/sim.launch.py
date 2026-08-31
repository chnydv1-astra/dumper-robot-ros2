from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": "-r empty.sdf"
        }.items()
    )

    robot_description = Command([
        "xacro ",
        PathJoinSubstitution([
            FindPackageShare("dumper_description"),
            "urdf",
            "dumper.urdf.xacro"
        ])
    ])

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "dumper",
            "-topic", "robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.5"
        ],
        output="screen"
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
            "/world/empty/model/dumper/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/model/dumper/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V"
        ],
        remappings=[
            ("/world/empty/model/dumper/joint_state", "/joint_states"),
            ("/model/dumper/tf", "/tf")
        ],
        output="screen"
    )

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("dumper_description"),
        "config",
        "view_robot.rviz"
    ])

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        parameters=[{"use_sim_time": True}],
        arguments=["-d", rviz_config_file],
        output="screen"
    )

    delayed_spawn = TimerAction(
        period=3.0,
        actions=[spawn_robot]
    )

    delayed_bridge = TimerAction(
        period=5.0,
        actions=[bridge]
    )

    delayed_rviz = TimerAction(
        period=5.0,
        actions=[rviz_node]
    )

    return LaunchDescription([
        gazebo,
        robot_state_publisher,
        delayed_spawn,
        delayed_bridge,
        delayed_rviz
    ])