#! /usr/bin/env bash
#
# Space Robotics Competition 2: NASA JSC
# Copyright (c), 2019-2021 NASA-JSC. All Rights Reserved
#

# system setups
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/usr/share/gazebo/setup.sh"

# set up srcp2 extras
source ~/scripts/srcp2_setup.bash

# Source ROS Worspaces
source $HOME/ros_workspace/install/setup.bash
source $HOME/cmp_workspace/install/setup.bash

roslaunch scoot scoot.launch "mode:=man" "name:=small_scout_1"
# run example solution
rosrun scoot repl.py

exit $?
