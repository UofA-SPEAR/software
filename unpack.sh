#!/usr/bin/env bash

# NOT VERIFIED, DO NOT USE THIS

# Parse command line options
# Some dependencies only need to be installed on the rover
if [ "$1" == "dev" ]; then
    DEV=true
    echo dev
elif [ "$1" == "rover" ]; then
    ROVER=true
    echo rover
else
    echo "Usage: unpack.sh [dev|rover]"
    echo "Use dev if you're developing on your own computer"
    echo "Use rover if you're unpacking on the tx2 (this will install zed-ros-wrapper)"
    exit 1
fi

###### Get script location
# https://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"

###### Update/ install git submodules
git submodule update --init --recursive

###### Setup ros path
cd ~
mkdir -p ~/ros/src
cd ~/ros
catkin build
# Add a line in .bashrc to source ros setup script
# Uses grep to check if the line already exists
if ! grep -Fxq "source ~/ros/devel/setup.bash" ~/.bashrc
then
    echo "source ~/ros/devel/setup.bash" >> ~/.bashrc
    source ~/ros/devel/setup.bash
fi

# Add our models to GAZEBO_MODEL_PATH
if ! grep -Fxq "export GAZEBO_MODEL_PATH=GAZEBO_MODEL_PATH:${DIR}/spear_simulator/models" ~/.bashrc
then
    echo "export GAZEBO_MODEL_PATH=GAZEBO_MODEL_PATH:${DIR}/spear_simulator/models" >> ~/.bashrc
    export GAZEBO_MODEL_PATH=GAZEBO_MODEL_PATH:$DIR/spear_simulator/models
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
    MonashUAS/canros
    gareth-cross/rviz_satellite
    FlexBE/flexbe_app
    eric-wieser/ros_numpy
    AIS-Bonn/nimbro_network
    UofA-SPEAR/uavcan_dsdl
    roboticsgroup/roboticsgroup_upatras_gazebo_plugins
)

cd ~/ros/src
for repo in ${repos[@]}; do
    echo $repo
    git clone https://github.com/$repo.git --depth=1 --recurse-submodules --jobs=12 &
done
wait
mv nimbro_network/*/ .
rm -rf nimbro_network
python -m pip install uavcan
mkdir -p ~/uavcan_vendor_specific_types
cd ~/uavcan_vendor_specific_types
ln -s ~/ros/src/uavcan_dsdl/spear
python -m pip install monotonic # Dependency for canros

###### Link our packages
# This needs to be moved all back into rover2 at some point
cd ~/ros/src
ln -s $DIR/spear_msgs
ln -s $DIR/spear_rover
ln -s $DIR/spear_station
ln -s $DIR/spear_simulator
ln -s $DIR/spear_behaviors
ln -s $DIR/case_moveit_config
ln -s $DIR/spear_util
ln -s $DIR/tests

###### Build everything
cd ~/ros
# update rosdeps first
rosdep install --from-paths src --ignore-src -r -y && \
catkin build --force-cmake && \

printf "Thanks for unpacking!\nNow that your enviroment is setup, you should never have to do this again.\nIf you are running this manually and have added a new package, please run the following command to make sure ROS sees it: source ~/ros/devel/setup.bash\n"
