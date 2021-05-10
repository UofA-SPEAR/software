#!/usr/bin/env bash
#
# Builds everything, while generating nicities like compile_commands.json,
# making build-time config more consistent, and so on.
#

# Parse options
COPY_HEADERS_AND_LIBS=$BUILD_COPY_HEADERS_AND_LIBS

ORIGINAL_DIR=$(pwd)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
SOURCE_DIR=${SCRIPT_DIR}/..

source ~/ros/devel/setup.bash

echo 'Changing directory to ros workspace.'
cd ~/ros

echo 'Building everything...'
catkin build --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=1

echo "Returning to ${ORIGINAL_DIR}."
cd "${ORIGINAL_DIR}"

echo "Copying ~/ros/build/compile_commands.json to ${SOURCE_DIR}/compile_commands.json (for code completion coolness)."

# Make a single, unified compile_commands.json from each package's compile_commands.json.
# From https://github.com/catkin/catkin_tools/issues/551#issuecomment-553521463
cd ~/ros
concatenated="build/compile_commands.json"
echo "[" > $concatenated
first=1
for d in build/*
do
    f="$d/compile_commands.json"
    if test -f "$f"; then
        if [ $first -eq 0 ]; then
            echo "," >> $concatenated
        fi
        cat $f | sed '1d;$d' >> $concatenated
    fi
    first=0
done
echo "]" >> $concatenated

cp ~/ros/build/compile_commands.json ${SOURCE_DIR}/compile_commands.original.json
if $COPY_HEADERS_AND_LIBS; then
    # We need to globally replace all occurences of /opt/ros with $DOCKER_HOST_REPO_LOCATION/.docker-opt-ros-mount,
    # and all occurences of /root/ros/devel/include with $DOCKER_HOST_REPO_LOCATION/.docker-root-ros-include-mount,
    # so that all ros-related headers and libs will be picked up by autocomplete engines.
    cp -r /opt/ros/* /tmp/docker-opt-ros-mount/
    cp -r /root/ros/devel/include/* /tmp/docker-root-ros-include-mount/
    cat ${SOURCE_DIR}/compile_commands.original.json \
        | sed -e "s@/opt/ros@${DOCKER_HOST_REPO_LOCATION}/\.docker-opt-ros-mount@g" \
        | sed -e "s@/root/ros/devel/include@${DOCKER_HOST_REPO_LOCATION}/\.docker-root-ros-include-mount@g" \
        | sed -e "s@/root/ros/src/spear_rover@${DOCKER_HOST_REPO_LOCATION}/pkg/spear_rover@g" \
        | sed -e "s@/root/ros/src/spear_simulator@${DOCKER_HOST_REPO_LOCATION}/pkg/spear_simulator@g" \
        | sed -e "s@/root/ros/src/spear_msgs@${DOCKER_HOST_REPO_LOCATION}/pkg/spear_msgs@g" \
        | sed -e "s@/root/ros/src/spear_station@${DOCKER_HOST_REPO_LOCATION}/pkg/spear_station@g" \
        > ${SOURCE_DIR}/compile_commands.json
else
    cp ${SOURCE_DIR}/compile_commands.original.json ${SOURCE_DIR}/compile_commands.json
fi
