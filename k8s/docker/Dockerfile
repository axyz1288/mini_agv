FROM axyz1288/aidocker:ros-arm64

# enable read write
RUN usermod -a -G dialout ${USERNAME}
ADD https://raw.githubusercontent.com/axyz1288/car/main/k8s/docker/car_entrypoint.sh /car_entrypoint.sh
RUN chmod 777 /car_entrypoint.sh
ENTRYPOINT [ "/usr/bin/tini-arm64", "--", "/car_entrypoint.sh" ]