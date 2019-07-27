[![Build Status](https://travis-ci.com/UofA-SPEAR/software.svg?branch=master)](https://travis-ci.com/UofA-SPEAR/software)

# Software

This is the repository for all our Linux-based software.
There are four ROS packages: spear_rover, spear_station, spear_msgs, and spear_simulator.
Each of these packages fulfills a specific role.

Usage guidelines and more detailed descriptions can be found in the READMEs of each respective package.

## spear_msgs

This is the package that contains all of the ROS messages defined by us, and for use between different packages.
This is a seperate package so that a node or package can depend on it without needing to build other packages.

## spear_rover

This package is for all of the code that runs the rover.
Essentially, any software that is supposed to run on the rover or is hardware-dependant (i.e. drive management or arm kinematics) should be placed in this package.

## spear_simulator

This package contains the code and configuration for the simulator.
This should ideally be able to run on the TX2, but it is not necessary for it to, as simulations can be run on more powerful computers.

## spear_station

This package contains all of the software that will be run at the base station during competition.
This includes our command and control interfaces and essentially anything that isn't run on the rover during competition.

# Setup and install instructions

## Install Ubuntu 16.04 desktop

We use ROS Kinetic which requires Ubuntu 16.04.
A virtual machine will work but a native install will run smoother, especially for the simulator.

You can find a .iso image of Ubuntu 16.04 [here](http://releases.ubuntu.com/16.04/).

## Install ROS

We use ROS for nearly everying on the rover.<br>
To install ROS, follow the instructions [here](http://wiki.ros.org/kinetic/Installation/Ubuntu).<br>
We recommend the "Desktop-Full Install" ROS package for best compatibility.

## Install other dependencies

Install the following dependencies:<br>


- x264: `apt-get install libx264-dev`
- Cython: `python -m pip install cython --user`
- Pygame: `python -m pip install pygame --user`
- Kivy: `python -m pip install kivy --user`
- Kivy Garden: `python -m pip install kivy-garden --user`
- Kivy Knob: `garden install knob`
- ROS Joy: `sudo apt-get install ros-kinetic-joy`
- libqt (required by nimbro_network): `apt-get install libqt4-dev`
- qmake (required by nimbro_network): `apt-get install qt4-qmake`
- ROS move\_base package: `apt-get install ros-kinetic-move-base`


Notes:
1. x264 is for encoding video and Kivy is for our user interface.
2. ROS Kinetic requires the Python 2 versions of all modules.

## Clone this repository and unpack

After installing the dependencies, clone this repo and run the `unpack.sh` script located within.

This will setup your catkin workspace for development.

You must `source ~/.bashrc` for the changes made by `unpack.sh` to take effect.

Run rosdep to install the required packages:

```
rosdep install --from-paths src --ignore-src -r -y
```

## Developing and building

The `unpack.sh` script will symlink all the source files to a catkin workspace located at `~/ros`.
You can work in this directory and when you need to build, either run `catkin_make` in the `~/ros` directory or simply run `./build.bash` from this directory which will handle things for you.

## “Permission Denied” errors when running roslaunch or roscore

Run the following 2 commands:
```
sudo rosdep fix-permissions
rosdep update
```

See this forum post for more info:
<https://answers.ros.org/question/60366/problem-with-roscore/>.


# Additional information

## Running "remote" launch files

Current status is: not working but close.

For now, just ssh into the remote and run the main launch file.
Then, on the local machine, run:

```
export ROS_MASTER_URI=http://tegra-ubuntu:11311/
```

Afterward, any nodes you run on your local machine will automatically connect to the remote machine.

See [this post](https://answers.ros.org/question/41446/a-is-not-in-your-ssh-known_hosts-file/).

## Code Formatting

### C++
```
clang-format-3.9 -i -style=Google <filename>
```

### Python
```
yapf -i <filename>
```

Note: we are linting for python 3.5.
