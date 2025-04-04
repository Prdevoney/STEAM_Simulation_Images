#!/bin/bash
set -e

# Step 1: Launch Gazebos websocket server
ign launch -v 4 /usr/share/ignition/ignition-launch5/configs/websocket.ign &

# Step 2: Launch ROS2 bridge (different for each simulation) 
# 2a:
source /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /world/world_demo/model/tugbot/link/scan_front/sensor/scan_front/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan &	
# 2b:
source /opt/ros/humble/setup.bash
run ros_ign_bridge parameter_bridge /model/tugbot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist &	

# Step 3: Start teleop twist 
python3 teleop_script/teleop_twist_keyboard.py &

# Step 4: CLI web socket server 
cd steam-websocket 
npm start &

# Step 5: Start HTTP health check server
cd .. 
cd steam-healthcheck
npm start &

tail -f /dev/null 
