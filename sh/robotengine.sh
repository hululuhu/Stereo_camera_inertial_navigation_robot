#!/bin/sh


expect -c "
set timeout 4
spawn su zhishan
expect \"Password:\"
send \"1ess!sm0re\r\"
send \"source /home/zhishan/.bashrc\r\"
send \"/home/zhishan/dashan_core &\r\"
send \"/home/zhishan/dashan_mode &\r\"
send \"/home/zhishan/dashan_serial &\r\"
send \"/home/zhishan/dashan_diagnosis &\r\"
send \"/home/zhishan/rec_rpc_server_node &\r\"
send \"/home/zhishan/odom_data &\r\"
send \"roslaunch dashan_bringup robotengine.launch\r\"
interact
"

