#!/usr/bin/env bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"

# this file is for spear specific environment variables and bash configuration

export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/root/ros/src/spear_simulator/models

exec "$@"
