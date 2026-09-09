# Dumper Robot ROS 2 & Gazebo Simulation

Autonomous dumper robot simulation in Gazebo Fortress with LiDAR SLAM mapping using `slam_toolbox`.

## Prerequisites
* ROS 2 (Humble/Iron/Rolling)
* Gazebo Fortress (`ros_gz`)
* `slam_toolbox`

## Quickstart

1. **Build the workspace:**
   ```bash
   colcon build --symlink-install
   source install/setup.bash

   ros2 launch dumper_gazebo sim.launch.py

   ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -p use_sim_time:=true \
  -p base_frame:=base_link \
  -p odom_frame:=odom

  ros2 run teleop_twist_keyboard teleop_twist_keyboard# dumper-robot-ros2

  Terminal 1 — Start the complete simulation
  cd ~/dev_dumper_ws
  source install/setup.bash
  ros2 launch dumper_gazebo full_system.launch.py

Terminal 2 — Start the collision predictor / LiDAR safety system
cd ~/dev_dumper_ws
source install/setup.bash
ros2 run collision_predictor collision_node

Terminal 3 — Drive Dumper1
cd ~/dev_dumper_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/dumper1/teleop_cmd_vel

Terminal 4 — Drive Dumper2
cd ~/dev_dumper_ws
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/dumper2/teleop_cmd_vel




  
