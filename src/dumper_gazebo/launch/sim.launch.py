from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    world_file = PathJoinSubstitution([
        FindPackageShare("dumper_gazebo"),
        "worlds",
        "mine_world.sdf"
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare("dumper_description"),
        "urdf",
        "dumper.urdf.xacro"
    ])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={"gz_args": ["-r ", world_file]}.items()
    )

    # ---------------- DUMPER 1 ----------------

    robot1_description = Command([
        "xacro ", xacro_file,
        " prefix:=dumper1/",
        " robot_namespace:=dumper1"
    ])

    robot1_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="dumper1",
        parameters=[{
            "robot_description": robot1_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    spawn_robot1 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "dumper1",
            "-topic", "/dumper1/robot_description",
            "-x", "0",
            "-y", "0",
            "-z", "0.5"
        ],
        output="screen"
    )

    # ---------------- DUMPER 2 ----------------

    robot2_description = Command([
        "xacro ", xacro_file,
        " prefix:=dumper2/",
        " robot_namespace:=dumper2"
    ])

    robot2_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="dumper2",
        parameters=[{
            "robot_description": robot2_description,
            "use_sim_time": True
        }],
        output="screen"
    )

    spawn_robot2 = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "dumper2",
            "-topic", "/dumper2/robot_description",
            "-x", "0",
            "-y", "35",
            "-z", "0.5",
            "-Y", "-1.5708"
        ],
        output="screen"
    )

        # ---------------- BRIDGE ----------------

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
                "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

                "/dumper1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/dumper1/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/dumper1/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                "/dumper1/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",

                "/dumper2/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                "/dumper2/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                "/dumper2/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",

            "/world/mine_world/model/dumper1/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/world/mine_world/model/dumper2/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model"

            
        ],
        remappings=[
            

            ("/world/mine_world/model/dumper1/joint_state",
             "/dumper1/joint_states"),

            ("/world/mine_world/model/dumper2/joint_state",
             "/dumper2/joint_states")

            
        ],
        output="screen"
    )
    # ---------------- RVIZ ----------------

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
        actions=[
            spawn_robot1,
            spawn_robot2
        ]
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
        robot1_state_publisher,
        robot2_state_publisher,
        delayed_spawn,
        delayed_bridge,
        delayed_rviz
    ])