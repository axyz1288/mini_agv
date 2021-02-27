FROM axyz1288/aidocker:ros-arm64

# enable read write
RUN usermod -a -G dialout ${USERNAME}
ADD https://raw.githubusercontent.com/axyz1288/mini_agv/main/k8s/docker/mini_agv_entrypoint.sh /mini_agv_entrypoint.sh
RUN chmod 777 /mini_agv_entrypoint.sh
ENTRYPOINT [ "/usr/bin/tini-arm64", "--", "/mini_agv_entrypoint.sh" ]
CMD [ "sleep", "infinity" ]