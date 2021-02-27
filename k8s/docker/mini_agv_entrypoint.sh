#!/bin/bash
set -e
if [ ! -f "~/.bashrc" ] 
then
	cp /etc/skel/.bashrc ~/
	echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
fi
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/slam && catkin_make && source ./devel/setup.bash
cd ~/mini_agv && catkin_make

roslaunch slamware_ros_sdk slamware_ros_sdk_server_node.launch ip_address:=192.168.11.1 &
exec "$@"