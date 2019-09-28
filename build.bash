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
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1 "$@"

echo "Returning to ${ORIGINAL_DIR}."
cd "${ORIGINAL_DIR}"

echo "Copying ~/ros/build/compile_commands.json to ${SCRIPT_DIR}/compile_commands.json (for code completion coolness)."
cp ~/ros/build/compile_commands.json ${SCRIPT_DIR}/compile_commands.json
