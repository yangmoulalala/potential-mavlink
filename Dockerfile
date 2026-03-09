FROM ros:humble-ros-base

RUN sudo apt update && \
    sudo apt install -y python3-pip curl wget htop vim unzip python3-rosdep ros-humble-ament-cmake && \
    pip install xmacro gdown


# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh

CMD [ "/bin/zsh" ]

# Install serial library (not available via rosdep in humble)
RUN git clone https://github.com/ZhaoXiangBox/serial.git && \
    cd serial && \
    mkdir build && \
    cd build && \
    . /opt/ros/humble/setup.sh && \
    cmake .. && \
    make && \
    sudo make install

# Clone and build mavlink_ws
RUN git clone https://github.com/yangmoulalala/mavlink_ws.git /root/mavlink_ws && \
    cd /root/mavlink_ws && \
    rosdep install --from-paths src --ignore-src -r -y && \
    . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install


# setup .zshrc
RUN echo 'export TERM=xterm-256color\n\
source ~/ros_ws/install/setup.zsh\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"\n'\
>> /root/.zshrc

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "/root/ros_ws/install/setup.bash"' \
      /ros_entrypoint.sh

RUN rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["ros2 run serial_mav serial_mav_node"]
