#!/usr/bin/env bash

ORIGINAL_DIR=$(pwd)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

source ~/ros/devel/setup.bash

echo 'Changing directory to ros workspace.'
cd ~/ros

echo 'Building everything...'
catkin_make -DCMAKE_EXPORT_COMPILE_COMMANDS=1

echo "Returning to ${ORIGINAL_DIR}."
cd "${ORIGINAL_DIR}"

# Check if we've symlinked the compile_commands.json file yet. If we haven't,
# then do it!
if [ ! -e "${SCRIPT_DIR}/compile_commands.json" ]; then
	echo "Symlinking compile_commands.json (for code completion coolness)."
	ln -s ~/ros/build/compile_commands.json ${SCRIPT_DIR}/compile_commands.json
fi
