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
catkin_make
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

###### Install nimbro networks
cd ~/ros/src
git clone https://github.com/AIS-Bonn/nimbro_network.git
mv nimbro_network/*/ .
rm -rf nimbro_network

###### Install ros wrapper package for zed camera
# This requires the ZED SDK to be installed
# ZED SDK requires CUDA
if [ $ROVER ]; then
    cd ~/ros/src
    git clone https://github.com/UofA-SPEAR/zed-ros-wrapper.git
    cd zed-ros-wrapper
    # Checkout the latest stable release of zed-ros-wrapper
    git checkout v2.7.x
    cd ~/ros/src
fi

###### Install canros
python -m pip install uavcan
cd ~/ros/src
git clone https://github.com/MonashUAS/canros.git

#### Get our DSDL definitions
cd ~
git clone https://github.com/UofA-SPEAR/uavcan_dsdl.git
cd uavcan_dsdl
git submodule update --init

###### Link our packages
# This needs to be moved all back into rover2 at some point
cd ~/ros/src
ln -s $DIR/spear_msgs
ln -s $DIR/spear_rover
ln -s $DIR/spear_station
ln -s $DIR/spear_simulator

###### Link UAVCAN DSDL definitions to the home directory.
mkdir -p ~/uavcan_vendor_specific_types
cd ~/uavcan_vendor_specific_types
ln -s ~/uavcan_dsdl/spear

###### Build everything
cd ~/ros
catkin_make --force

printf "Thanks for unpacking!\nNow that your enviroment is setup, you should never have to do this again.\nPlease run the following command to install the correct packages:\nrosdep install --from-paths src --ignore-src -r -y\n"
