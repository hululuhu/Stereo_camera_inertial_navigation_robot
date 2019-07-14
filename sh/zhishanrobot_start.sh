#!/bin/bash

source  /home/zhishan/Dashan_demo/sh/ip.sh
echo $MASTER_IP

source /home/zhishan/Dashan_demo/devel/setup.bash
export ROS_MASTER_URI=http://$MASTER_IP
roslaunch dashan_bringup zhishanrobot_start.launch 