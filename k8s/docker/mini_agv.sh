#!/bin/bash
set -e
git clone https://github.com/axyz1288/mini_agv.git ~/mini_agv
chmod 777 -R ~/mini_agv
source ~/slam/devel/setup.bash
cd ~/mini_agv && catkin_make

roslaunch slamware_ros_sdk slamware_ros_sdk_server_node.launch ip_address:=192.168.11.1 nodename:=${HOSTNAME} &
~/mini_agv/build/agv ${HOSTNAME} "$1" "$2"