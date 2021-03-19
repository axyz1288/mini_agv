FROM axyz1288/aidocker:ros-arm64

# enable read write
RUN usermod -a -G dialout ${USERNAME}
RUN curl https://raw.githubusercontent.com/axyz1288/mini_agv/main/k8s/docker/ros_entrypoint.sh > /ros_entrypoint.sh && \
    curl https://raw.githubusercontent.com/axyz1288/mini_agv/main/k8s/docker/mini_agv.sh > /mini_agv.sh && \
    curl https://raw.githubusercontent.com/axyz1288/mini_agv/main/k8s/docker/update_slam.sh > /update_slam.sh && \
    chmod a+x /ros_entrypoint.sh /mini_agv.sh /update_slam.sh
ENTRYPOINT [ "/usr/bin/tini-arm64", "--", "/ros_entrypoint.sh" ]
CMD [ "sleep", "infinity" ]