FROM axyz1288/aidocker:ros-arm64

# enable read write
RUN usermod -a -G dialout ${USERNAME} \
    && git clone https://github.com/axyz1288/car.git /home/${USERNAME}/car \
    && chmod 777 -R /home/${USERNAME}/car

ADD https://raw.githubusercontent.com/axyz1288/car/main/k8s/docker/car/car_entrypoint.sh /car_entrypoint.sh
RUN chmod 777 /car_entrypoint.sh
ENTRYPOINT [ "/usr/bin/tini-arm64", "--", "/car_entrypoint.sh" ]
CMD [ "bash" ]