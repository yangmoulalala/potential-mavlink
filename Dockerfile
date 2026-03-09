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

# source entrypoint setup
RUN sed --in-place --expression \
      '$isource "/root/mavlink_ws/install/setup.bash"' \
      /ros_entrypoint.sh

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

# Create a startup script that properly sources the workspace and runs the node
RUN echo '#!/bin/bash\n\
cd /root/mavlink_ws\n\
source /opt/ros/humble/setup.bash\n\
source install/setup.bash\n\
exec ros2 run serial_mav serial_mav_node "$@"' > /root/start_serial_node.sh && \
    chmod +x /root/start_serial_node.sh

# Set zsh as default shell
RUN usermod -s /bin/zsh root

RUN rm -rf /var/lib/apt/lists/*

# Use the standard ROS entrypoint and our custom startup script
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["/root/start_serial_node.sh"]