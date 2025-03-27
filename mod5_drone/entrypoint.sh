#!/bin/bash
set -e

# Step 1: Start start simulation
ign gazebo -v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/multicoper_velocity_control.sdf -s &

# Step 2: Launch Gazebos websocket server
ign launch -v 4 /usr/share/ignition/ignition-launch5/configs/websocket.ign &

# Step 3: Launch ROS2 bridge (different for each simulation) 
# 3a: 
soure /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /X4/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist  &	
# 3b:
soure /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /X3/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist &
# 3c:
soure /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /world/multicopter/model/X4/link/base_link/sensor/camera_front/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo &

# Step 4: CLI web socket server 
cd steam-websocket 
npm start &

tail -f /dev/null 
