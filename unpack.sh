#!/usr/bin/env bash
###### Get script location
# https://stackoverflow.com/a/246128
SOURCE="${BASH_SOURCE[0]}"
while [ -h "$SOURCE" ]; do # resolve $SOURCE until the file is no longer a symlink
  DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"
  SOURCE="$(readlink "$SOURCE")"
  [[ $SOURCE != /* ]] && SOURCE="$DIR/$SOURCE" # if $SOURCE was a relative symlink, we need to resolve it relative to the path where the symlink file was located
done
DIR="$( cd -P "$( dirname "$SOURCE" )" >/dev/null && pwd )"

###### Setup ros path
cd ~
mkdir -p ~/ros/src
cd ~/ros
catkin_make
echo "source ~/ros/devel/setup.bash" >> ~/.bashrc
source ~/ros/devel/setup.bash

###### Install nimbro networks
cd ~/ros/src
git clone https://github.com/AIS-Bonn/nimbro_network.git
mv nimbro_network/*/ .
rm -rf nimbro_network

###### Install canros
python -m pip install uavcan
git clone https://github.com/MonashUAS/canros.git

###### Update/ install git submodules
git submodule update --init --recursive

###### Link our packages
ln -s $DIR/drive_system
ln -s $DIR/rover2
ln -s $DIR/rover2_can

###### Link UAVCAN DSDL definitions to the home directory.
mkdir -p ~/uavcan_vendor_specific_types
cd ~/uavcan_vendor_specific_types
ln -s $DIR/rover2_can/uavcan_dsdl/spear

###### Build everything
cd ~/ros
catkin_make
