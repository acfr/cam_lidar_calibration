#!/bin/bash

source /opt/ros/melodic/setup.bash
catkin build
source /catkin_ws/devel/setup.bash
exec "$@"