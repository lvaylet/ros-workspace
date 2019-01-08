#!/bin/bash

# Due to the automated nature of ROSLaunch, it avoids executing the user's .bashrc or .zshrc on remote nodes and requires an environment file to assign remote environment variables and load the packages from the workspace.
#
# Reference:
# https://github.com/pandora-auth-ros-pkg/pandora_docs/wiki/Remote-Machines-Running-ROS-nodes

source /home/laurent/catkin_ws/devel/setup.bash

exec "$@"
