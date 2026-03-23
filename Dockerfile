FROM ros:humble-ros-base

# 安装环境
RUN sudo apt update && \
    sudo apt install -y python3-pip curl wget ros-humble-ament-cmake ros-humble-tf-transformations && \
    sudo pip3 install --upgrade pip && \
    sudo pip3 install rosdepc pyserial

# 更新~/.bashrc
RUN echo '\nsource /opt/ros/humble/setup.bash\n\
source /app/install/setup.bash' >> /root/.bashrc

# 设置工作目录和拷贝文件
WORKDIR /app
COPY ./src /app/src
COPY ./start.sh /app/start.sh
RUN chmod +x /app/start.sh
ENTRYPOINT ["/bin/bash", "/app/start.sh"]

# 安装依赖
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    rosdepc install --from-paths src --ignore-src -r -y"
