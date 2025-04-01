#!/bin/bash
set -e

# Step 1: Launch Gazebos websocket server
ign launch -v 4 /usr/share/ignition/ignition-launch5/configs/websocket.ign &

# Step 2: Launch ROS2 bridge (different for each simulation) 
source /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /model/vehicle_blue/battery/linear_battery/state@sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState  &

# Step 3: CLI web socket server 
cd steam-websocket 
npm start &

# Step 4: Start HTTP health check server
cd .. 
cd steam-healthcheck
npm start &

tail -f /dev/null 
