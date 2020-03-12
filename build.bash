#!/usr/bin/env bash
#
# Builds everything, while generating nicities like compile_commands.json,
# making build-time config more consistent, and so on.
#
# All arguments passed to this script will be passed to catkin_make.

ORIGINAL_DIR=$(pwd)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

source ~/ros/devel/setup.bash

echo 'Changing directory to ros workspace.'
cd ~/ros

echo 'Building everything...'
catkin build -p 1 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1 "$@"

echo "Returning to ${ORIGINAL_DIR}."
cd "${ORIGINAL_DIR}"

echo "Copying ~/ros/build/compile_commands.json to ${SCRIPT_DIR}/compile_commands.json (for code completion coolness)."
# We need to globally replace all occurences of /opt/ros with $DOCKER_HOST_REPO_LOCATION/.docker-opt-ros-mount,
# and all occurences of /root/ros/devel/include with $DOCKER_HOST_REPO_LOCATION/.docker-root-ros-include-mount,
# so that all ros-related headers and libs will be picked up by autocomplete engines.
cp ~/ros/build/compile_commands.json ${SCRIPT_DIR}/compile_commands.original.json
cp -r /opt/ros/* /tmp/docker-opt-ros-mount/
cp -r /root/ros/devel/include/* /tmp/docker-root-ros-include-mount/
cat ${SCRIPT_DIR}/compile_commands.original.json \
	| sed -e "s@/opt/ros@${DOCKER_HOST_REPO_LOCATION}/\.docker-opt-ros-mount@g" \
	| sed -e "s@/root/ros/devel/include@${DOCKER_HOST_REPO_LOCATION}/\.docker-root-ros-include-mount@g" \
	| sed -e "s@/root/ros/src/spear_rover@${DOCKER_HOST_REPO_LOCATION}/spear_rover@g" \
	| sed -e "s@/root/ros/src/spear_simulator@${DOCKER_HOST_REPO_LOCATION}/spear_simulator@g" \
	| sed -e "s@/root/ros/src/spear_msgs@${DOCKER_HOST_REPO_LOCATION}/spear_msgs@g" \
	| sed -e "s@/root/ros/src/spear_station@${DOCKER_HOST_REPO_LOCATION}/spear_station@g" \
	> ${SCRIPT_DIR}/compile_commands.json
