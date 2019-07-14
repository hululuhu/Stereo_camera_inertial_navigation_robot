#!/bin/bash

source  ./sh/ip.sh
echo $MASTER_IP

source ./devel/setup.bash
export ROS_HOSTNAME=hulu-E6400
export ROS_MASTER_URI=http://$MASTER_IP

roslaunch ros_joy teleop_joy.launch 