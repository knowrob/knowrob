#!/bin/bash

source /home/ros/devel/setup.bash
export ROS_PACKAGE_PATH="$ROS_PACKAGE_PATH:/home/ros/user_data/:/openease_ws"

roslaunch /home/ros/src/knowrob/launch/openease.launch
