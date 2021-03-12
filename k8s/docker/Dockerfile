FROM axyz1288/aidocker:ros-arm64

# enable read write
RUN usermod -a -G dialout ${USERNAME}
ADD https://raw.githubusercontent.com/axyz1288/mini_agv/main/k8s/docker/ros_entrypoint.sh /ros_entrypoint.sh
ADD https://raw.githubusercontent.com/axyz1288/mini_agv/main/k8s/docker/mini_agv.sh /mini_agv.sh
RUN chmod 777 /ros_entrypoint.sh
RUN chmod 777 /mini_agv.sh
ENTRYPOINT [ "/usr/bin/tini-arm64", "--", "/ros_entrypoint.sh" ]
CMD [ "sleep", "infinity" ]