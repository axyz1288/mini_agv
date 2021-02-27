FROM axyz1288/aidocker:ros-arm64

RUN git clone https://github.com/axyz1288/mini_agv.git /home/${USERNAME}/mini_agv \
    && tar -C /home/${USERNAME} -Jxf /home/${USERNAME}/mini_agv/slam.tar.xz \
    && chmod 777 -R /home/${USERNAME}/mini_agv \
    && chmod 777 -R /home/${USERNAME}/slam

# enable read write
RUN usermod -a -G dialout ${USERNAME}
ADD https://raw.githubusercontent.com/axyz1288/mini_agv/main/k8s/docker/mini_agv_entrypoint.sh /mini_agv_entrypoint.sh
RUN chmod 777 /mini_agv_entrypoint.sh
ENTRYPOINT [ "/usr/bin/tini-arm64", "--", "/mini_agv_entrypoint.sh" ]
CMD [ "sleep", "infinity" ]