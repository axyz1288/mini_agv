#!/bin/bash
set -e
if [ ! -f "~/.bashrc" ] 
then
	cp /etc/skel/.bashrc ~/
	echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
fi
if [ -f "~/car" ]
then
	rm -rf ~/car
fi
source /opt/ros/$ROS_DISTRO/setup.bash
git clone https://github.com/axyz1288/car.git ~/car
cd ~/car && catkin_make
exec "$@"
