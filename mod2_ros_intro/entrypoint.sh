#!/bin/bash
set -e

# Step 1: Start start simulation
ign gazebo -v 4 -r /usr/share/ignition/ignition-gazebo6/worlds/linear_battery_state.sdf -s &

# Step 2: Launch Gazebos websocket server
ign launch -v 4 /usr/share/ignition/ignition-launch5/configs/websocket.ign &

# Step 3: Launch ROS2 bridge (different for each simulation) 
# 3a: 
source /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /model/vehicle_blue/battery/linear_battery/state@sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState  &
		
# 3b:
source /opt/ros/humble/setup.bash
ros2 run ros_ign_bridge parameter_bridge /model/vehicle_blue/battery/linear_battery/state@sensor_msgs/msg/BatteryState@ignition.msgs.BatteryState &

# Step 4: CLI web socket server 
cd steam-websocket 
npm start &

tail -f /dev/null 
