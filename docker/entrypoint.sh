#!/bin/bash
set -e

source /opt/ros/jazzy/setup.bash

if [ -f "/home/user/autonomy_stack_ros_jazzy/install/setup.bash" ]; then
    source /home/user/autonomy_stack_ros_jazzy/install/setup.bash
fi

exec "$@"
