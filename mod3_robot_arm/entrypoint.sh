#!/bin/bash
set -e

# Step 1: Start start simulation
cd gaz_worlds_files/
ign gazebo -v 4 turtlebot3_baylands.sdf -s &

# Step 2: Launch Gazebos websocket server
cd ..
ign launch -v 4 /usr/share/ignition/ignition-launch5/configs/websocket.ign &

# Step 3: Launch ROS2 bridge (different for each simulation) 
ros2 run ros_ign_bridge parameter_bridge /model/turtlebot3_burger/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist &
		
# Step 4: CLI web socket server 
cd steam-websocket 
npm start &

tail -f /dev/null 
