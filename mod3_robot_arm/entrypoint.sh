#!/bin/bash
set -e

# Step 1: Start start simulation
ign gazebo -v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/joint_trajectory_controller.sdf -s &

# Step 2: Launch Gazebos websocket server
ign launch -v 4 /usr/share/ignition/ignition-launch5/configs/websocket.ign &

# Step 3: Launch ROS2 bridge (different for each simulation) 
# 3a: Joint Trajectory Controller
source /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /model/RR_position_control/joint_trajectory@trajectory_msgs/msg/JointTrajectory@ignition.msgs.JointTrajectory &
# 3b: Velocity Controller
source /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /model/RR_velocity_control/joint_trajectory@trajectory_msgs/msg/JointTrajectory@ignition.msgs.JointTrajectory & 
# 3c: Effort Controller
source /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /custom_topic_effort_control@trajectory_msgs/msg/JointTrajectory@ignition.msgs.JointTrajectory &

# Step 4: CLI web socket server 
cd steam-websocket 
npm start &

# Step 5: Start HTTP health check server
cd .. 
cd steam-healthcheck
npm start &

tail -f /dev/null 
