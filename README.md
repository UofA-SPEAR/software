[![Build Status](https://travis-ci.com/UofA-SPEAR/software.svg?branch=master)](https://travis-ci.com/UofA-SPEAR/software)

# Software

This is the repository for all Linux-based software.
There are four ROS packages, spear_rover, spear_station, spear_msgs,
and spear_simulator. Each of these packages fulfills a specific role.

Usage guidelines, naming, and more detailed divisions will be placed in the README's of each
respective package.

## spear_rover ##

This package is for all of the code that runs the rover. Essentially
any software that is supposed to run on the rover, or is hardware-dependant
(i.e. drive management or arm kinematics) should be placed in this package.

## spear_station ##

This package contains all of the software that will be run at the base station, during competition.
This means stuff like command & control interfaces, or essentially anything that isn't run on the rover
during competition.

## spear_simulator ##

This package contains the code and configuration for the simulator. This should ideally be
able to run on the TX2, but it is not necessary, as all of the simulation can be run on people's
laptops or other, more powerful computers.

## spear_msgs ##

This is the package that contains all of the ROS messages defined by us, and for use between different packages.
This is a seperate package so that a node or package can depend on it, while not needing to build the nodes
required from other packages.

# Migration TODO

- [ ] Fix travisCI
- [ ] find put where the drive_commands send_ros.py file went
- [ ] Add dependency info to this README

# Setup and Install Instructions

## Install Ubuntu 16.04 desktop
We use ROS Kinetic which requires Ubuntu 16.04. A virtual machine will work but a native install will run smoother, especially for the simulator.

You can find a .iso image of Ubuntu 16.04 [here](http://releases.ubuntu.com/16.04/).

## Install ROS
We use ROS we nearly everying on the rover. To install ROS, follow the instructions
[here](http://wiki.ros.org/kinetic/Installation/Ubuntu) <br />
We recommend the "Desktop-Full Install" ROS package for best compatibility.

## Install Other Dependencies

Install the following dependencies. Note: x264 is for encoding video and Kivy is for our user interface.

x264: `$ apt-get install libx264-dev`<br>
Cython: `$ python -m pip install cython --user`<br>
Pygame: `$ python -m pip install pygame --user`<br>
Kivy: `$ python -m pip install kivy --user`<br>
Kivy Garden: `$ python -m pip install kivy-garden --user`<br>
Kivy Knob: `$ garden install knob`<br>
ROS Joy: `$ sudo apt-get install ros-kinetic-joy`<br>
libqt (required by Nimbro): `$ apt-get install libqt4-dev`<br>
qmake (required by Nimbro): `$ apt-get install qt4-qmake`<br>

Note: ROS Kinetic requires the python 2 versions of all modules.

## Clone this repository and unpack
After installing dependencies, clone this repo and run the `unpack.sh` command located within.

This will setup your catkin workspace for development.

You must `source ~/.bashrc` for the changes made by `unpack.sh` to take effect.

## “Permission Denied” errors when running roslaunch or roscore
Run the following 2 commands:
`sudo rosdep fix-permissions`
`rosdep update`

See this forum post for more info:
<https://answers.ros.org/question/60366/problem-with-roscore/>


# Stuff to keep, not sure where to put it #

## Running "remote" launch files ##

Current status is: not working but close.

For now, just ssh into the remote and run the main launch file.
Then on the local machine, run:

```
export ROS_MASTER_URI=http://tegra-ubuntu:11311/
```

And then any nodes you run on your local machine will automatically
connect to the remote machine.

### How To (not working yet) ###

See [this post](https://answers.ros.org/question/41446/a-is-not-in-your-ssh-known_hosts-file/)

## Code Formatting ##

### C++ ###
```
clang-format-3.9 -i -style=Google <filename>
```

### Python ###
```
yapf -i <filename>
```
