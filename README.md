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
