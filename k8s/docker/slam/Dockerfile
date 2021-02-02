FROM axyz1288/car

ADD https://raw.githubusercontent.com/axyz1288/car/main/k8s/docker/slam/slam_entrypoint.sh /slam_entrypoint.sh
RUN chmod 777 /slam_entrypoint.sh
ENTRYPOINT ["/usr/bin/tini-arm64", "--", "/slam_entrypoint.sh"]
CMD ["roslaunch", "slamware_ros_sdk", "slamware_ros_sdk_server_node.launch", "ip_address:=192.168.11.1"]