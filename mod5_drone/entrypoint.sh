#!/bin/bash
set -e

# Step 1: Launch Gazebos websocket server
ign launch -v 4 /usr/share/ignition/ignition-launch5/configs/websocket.ign &

# Step 2: Launch ROS2 bridge (different for each simulation) 
# 2a: 
source /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /X4/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist  &	
# 2b:
source /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /X3/gazebo/command/twist@geometry_msgs/msg/Twist@ignition.msgs.Twist &
# 2c:
source /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /world/multicopter/model/X4/link/base_link/sensor/camera_front/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo &

# Step 3: CLI web socket server 
cd steam-websocket 
npm start &

# Step 4: Start HTTP health check server
cd .. 
cd steam-healthcheck
npm start &

tail -f /dev/null 
