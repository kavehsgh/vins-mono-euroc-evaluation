#!/bin/bash
source /root/catkin_ws/devel/setup.bash

# 1. Start VINS-Mono with custom config
# We use the config_overrides folder which is mapped to ./config on host
echo "Starting VINS-Mono..."
roslaunch vins_estimator euroc.launch config_path:=/root/catkin_ws/config_overrides/euroc_tuned.yaml &
VINS_PID=$!
sleep 5

# 2. Start Rviz
echo "Starting Rviz..."
roslaunch vins_estimator vins_rviz.launch &
RVIZ_PID=$!
sleep 5

# 3. Play Bag
echo "Playing Dataset..."
rosbag play /root/catkin_ws/data/MH_01_easy.bag

# Cleanup
kill $VINS_PID
kill $RVIZ_PID
