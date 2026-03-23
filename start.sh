#!/bin/bash
set -e

echo "potential-mavlink start"

echo "ROS_DOMAIN_ID=$ROS_DOMAIN_ID"

# source ros
source /opt/ros/humble/setup.bash

# build workspace
colcon build --symlink-install

# source workspace
source install/setup.bash

# run node
ros2 run mavlink mavlink_node
