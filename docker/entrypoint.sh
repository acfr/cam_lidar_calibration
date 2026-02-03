#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/jazzy/setup.bash

# Source workspace if built
if [ -f "/ros2_ws/install/setup.bash" ]; then
    echo "Sourcing workspace..."
    source /ros2_ws/install/setup.bash
fi

# Execute the command
exec "$@"
