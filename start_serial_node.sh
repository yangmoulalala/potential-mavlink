#!/bin/bash
set -e

echo "====== ROS2 MAVLink Serial Container Start ======"

cd /root/mavlink_ws

# source ros
source /opt/ros/humble/setup.bash

# build workspace
colcon build --symlink-install

# source workspace
source install/setup.bash

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# run node
# exec ros2 run serial_mav serial_mav_node "$@"
exec ros2 run mavlink mavlink_node "$@"
