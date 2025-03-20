#!/bin/bash
set -e

# Step 1: Start Flask app
python3 app.py &

# Step 3: Start start simulation
cd .. 
cd gaz_worlds_files/
ign gazebo -v 4 turtlebot3_baylands.sdf -s &

# Step 4: Launch Gazebos websocket server
cd ..
ign launch -v 4 /usr/share/ignition/ignition-launch5/configs/websocket.ign &

# Step 5: CLI web socket server for Xterm.js (most likely)


tail -f /dev/null 
