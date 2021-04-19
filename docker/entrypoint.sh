#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /SC2/srcp2-competitors/ros_workspace/install/setup.bash
source /SC2/devel/setup.bash

export ROS_MASTER_URI=http://simmaster:11311

exec "$@"
