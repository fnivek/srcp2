#!/bin/bash

echo "This file must be sourced not run"
# Source ros
source /opt/ros/${ROS_DISTRO}/setup.bash

# Set the number of jobs to use when building
NUM_JOBS=$1
if [[ -z "${NUM_JOBS}" ]]; then
    NUM_JOBS=16
fi
catkin config -w /home/user/catkin_ws/ --init --make-args -j${NUM_JOBS}
