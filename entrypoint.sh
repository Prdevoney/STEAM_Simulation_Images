#!/bin/bash
set -e

echo "starting setup!..."

# source Gazebo and ROS
echo "sourcing Gazebo and ROS..."
source /opt/ros/noetic/setup.bash
source /usr/share/gazebo/setup.sh

echo "getting GZweb..."
cd /root/gzweb
echo "running npm deploy..."
./deploy.sh -m local

echo "starting flask app..."
cd /root
python3 app.py &
FLASK_PID=$!

echo "starting gzweb..."
cd /root/gzweb
npm start &
GZWEB_PID=$!


# launch the simulation
echo "launching simulation..."
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch gui:=false
GAZEBO_PID=$!

echo "all processes started!"
wait $FLASK_PID $GZWEB_PID $GAZEBO_PID

#tail -f /dev/null