FROM axyz1288/aidocker:ros-arm64

# Install realsense2
RUN apt-key adv --keyserver keys.gnupg.net --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE \
    && add-apt-repository "deb http://realsense-hw-public.s3.amazonaws.com/Debian/apt-repo bionic main" -u \
    && apt-get -y update && apt-get -y install --no-install-recommends \
    librealsense2-utils \
    librealsense2-dev \
    && rm -rf /var/lib/apt/lists/*

# enable read write
RUN usermod -a -G dialout ${USERNAME} \
    && git clone https://github.com/axyz1288/car.git /home/${USERNAME}/car \
    && chmod 777 -R /home/${USERNAME}/car

ADD https://raw.githubusercontent.com/axyz1288/car/main/k8s/docker/car_entrypoint.sh /car_entrypoint.sh
RUN chmod a+x /car_entrypoint.sh
ENTRYPOINT [ "/usr/bin/tini-arm64", "--", "/car_entrypoint.sh" ]
CMD [ "bash" ]