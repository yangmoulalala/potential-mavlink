FROM ros:humble-ros-base

RUN sudo apt update && \
    sudo apt install -y python3-pip curl wget python3-rosdep ros-humble-ament-cmake


# setup zsh
RUN sh -c "$(wget -O- https://github.com/deluan/zsh-in-docker/releases/download/v1.2.1/zsh-in-docker.sh)" -- \
    -t jispwoso -p git \
    -p https://github.com/zsh-users/zsh-autosuggestions \
    -p https://github.com/zsh-users/zsh-syntax-highlighting && \
    chsh -s /bin/zsh

# Install serial library (not available via rosdep in humble)
RUN git clone https://github.com/ZhaoXiangBox/serial.git && \
    cd serial && \
    mkdir build && \
    cd build && \
    . /opt/ros/humble/setup.sh && \
    cmake .. && \
    make && \
    sudo make install

# Append ROS environment to .zshrc (append to existing zsh config)
RUN echo '\n# ROS 2 Environment\n\
source /opt/ros/humble/setup.zsh\n\
source /root/mavlink_ws/install/setup.zsh\n\
eval "$(register-python-argcomplete3 ros2)"\n\
eval "$(register-python-argcomplete3 colcon)"' >> /root/.zshrc

# Also setup .bashrc for compatibility
RUN echo '\n# ROS 2 Environment\n\
source /opt/ros/humble/setup.bash\n\
source /root/mavlink_ws/install/setup.bash' >> /root/.bashrc

# workspace
WORKDIR /root/mavlink_ws

# 复制整个仓库
COPY . /root/mavlink_ws

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdep install --from-paths src --ignore-src -r -y && \
    colcon build --symlink-install"

RUN chmod +x /root/mavlink_ws/start_serial_node.sh

ENTRYPOINT ["/root/mavlink_ws/start_serial_node.sh"]

# Set zsh as default shell
RUN usermod -s /bin/zsh root

RUN rm -rf /var/lib/apt/lists/*