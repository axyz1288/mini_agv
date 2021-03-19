#!/bin/bash
set -e
rm -rf ~/slam/*
wget https://github.com/axyz1288/slam/raw/main/slam.tar.xz
tar -C ~ -Jxf ~/slam.tar.xz
chmod 777 -R ~/slam
cd ~/slam && catkin_make
chmod a+x -R ~/slam/devel/lib/slamware_ros_sdk/slamware_ros_sdk_server_node
exec "$@"