#!/bin/bash
set -e
if [ ! -f "~/.bashrc" ] 
then
	cp /etc/skel/.bashrc ~/
	echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
fi
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/slamware_ros_sdk_linux-aarch64-gcc5.4 && catkin_make
source ./devel/setup.bash
exec "$@"