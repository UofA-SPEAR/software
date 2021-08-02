#!/usr/bin/env bash

# Parse options
if [ $UNPACK_ENV == "dev" ]; then
    DEV=true
    echo dev
elif [ $UNPACK_ENV == "rover" ]; then
    ROVER=true
    echo rover
else
    echo "UNPACK_ENV should be set to one of rover or dev"
    exit 1
fi
SKIP_BUILD=$UNPACK_SKIP_BUILD

###### Get script location
# https://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE/" )/.." >/dev/null && pwd )"

###### Setup ros path
cd ~
mkdir -p ~/ros/src
cd ~/ros
if ! $SKIP_BUILD; then
    catkin build
fi
# Add a line in .bashrc to source ros setup script
# Uses grep to check if the line already exists
if ! grep -Fxq "source ~/ros/devel/setup.bash" ~/.bashrc
then
    echo "source ~/ros/devel/setup.bash" >> ~/.bashrc
    source ~/ros/devel/setup.bash
fi

# Add our models to GAZEBO_MODEL_PATH
MODEL_PATH=$DIR/pkg/spear_simulator/models
if ! grep -Fxq "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$MODEL_PATH" ~/.bashrc
then
    echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:$MODEL_PATH" >> ~/.bashrc
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$MODEL_PATH
fi

# Add rosdep alias to make things nicer
echo "alias update_rosdeps='cd ~/ros && rosdep install --from-paths src --ignore-src -r -y'" >> ~/.bashrc

###### Install ros wrapper package for zed camera
# This requires the ZED SDK to be installed
# ZED SDK requires CUDA
# TODO: this is still not working properly for some reason
if [ $ROVER ]; then
    cd ~/ros/src
    rm -rf zed-ros-wrapper
    curl -o zed-ros-wrapper.tar.gz https://codeload.github.com/stereolabs/zed-ros-wrapper/tar.gz/v3.0.0
    tar xzf zed-ros-wrapper.tar.gz
    mv zed-ros-wrapper-3.0.0 zed-ros-wrapper
    rm zed-ros-wrapper.tar.gz
fi

###### Install various dependencies from source

repos=(
    eric-wieser/ros_numpy:master
    FlexBE/flexbe_app:master
    gareth-cross/rviz_satellite:master
    machinekoder/ar_track_alvar:melodic-devel
    roboticsgroup/roboticsgroup_upatras_gazebo_plugins:master
    UofA-SPEAR/nimbro_network:master
    UofA-SPEAR/uavcan_dsdl:master
    UofA-SPEAR/zed-models:main
)

cd ~/ros/src
for item in ${repos[@]}; do
    IFS=':' read repo branch <<< "${item}"
    echo "$repo ($branch)"
    git clone --branch $branch https://github.com/$repo.git --depth=1 --recurse-submodules --jobs=12 &
done
wait
pip3 install uavcan
mkdir -p ~/uavcan_vendor_specific_types
cd ~/uavcan_vendor_specific_types
ln -s ~/ros/src/uavcan_dsdl/spear

###### Link our packages
# This needs to be moved all back into rover2 at some point
cd ~/ros/src
find $DIR/pkg -mindepth 1 -maxdepth 1 -type d -exec ln -s {} ';'

###### Build everything
set -e
cd ~/ros
# update rosdeps first
rosdep install --from-paths src --ignore-src -r -y
if ! $SKIP_BUILD; then
    catkin build --force-cmake
fi

printf "Thanks for unpacking!\nNow that your enviroment is setup, you should never have to do this again.\nIf you are running this manually and have added a new package, please run the following command to make sure ROS sees it: source ~/ros/devel/setup.bash\n"
