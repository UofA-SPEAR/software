# Artificial intelligence simulator
For training our Mars rover, we have decided to create a simulator that allows us to drive around a 3D model of our rover in a virtual world. This virtual world is designed to be similar to the types of environments we may encounter in competition.

## Dependencies

You may need to install the following packages using apt:
```
sudo apt-get install ros-kinetic-controller-manager
sudo apt-get install ros-kinetic-gazebo-ros-control
sudo apt-get install ros-kinetic-diff-drive-controller
sudo apt-get install ros-kinetic-hector-gazebo-plugins
```
If ROS still can't find the above packages, you may need to install them using rosdep.

## Running with ROS
The gazebo models and launch files for our rover have been wrapped in a ROS package called "spear_simulator". After cloning this repository, run the "unpack.sh" script. This will set up the ros package. Then, you should be able to launch our rover model in a gazebo simulation using "roslaunch spear_simulator diffdrive.launch".

## Getting started
To run our simulator, several dependencies will need to be installed, mainly ROS and Gazebo. We recommend additionally setting up a virtual machine to run our simulator, as it is quite invasive to install on your system and requires that you use Ubuntu Xenial (16.04 LTS). Feel free to use either VirtualBox or VMWare.

Currently our team uses ROS Kinetic Kame, which can be found at http://wiki.ros.org/kinetic/Installation. We recommend the “Desktop-Full Install” ROS package for best compatibility.

Install Gazebo 7, which we use as our physics engine and 3D simulator: http://gazebosim.org/download. Feel free to experiment with creating 3D models and environments to get a feel for the program.

For the most part, the files stored in this repository will be 3D models used by Gazebo. Copying them to `~/.gazebo/models` will allow you to add them to a simulation using Gazebo's insert menu.

Completed worlds are also stored in this repository. To run a world in gazebo use: `gazebo <relative_path_to_file>`.

## 3D models
NOTE: ROS kinetic uses Gazebo 7. This version of Gazebo only supports .dae, .stl, and .svg model formats. If you are adding a new model to this repo, it must be in one of the formats specified above. If your 3D model is not in one of the formats above, consider using Blender to convert your file to a .dae (the easiest format to convert to).

This repo contains many object that will help the rover learn. From obstacles to waypoints, objects contained in this repo will provide data for the rover to improve the way it drives. Right now we use tennis balls as waypoints, as they are easy for us to acquire in real life and simplify our image recognition code. There are multiple variation of tennis balls located in this repo to test the accuracy of the rover's ability to recognize tennis balls.

## Rover model
In the models folder of this repo is a TARS rover 3D model. Using roslaunch, a fully functioning virtual version of the rover can be used for our simulations.

If you would like to view a static version of TARS, simply follow the instructions in the 3D models section to add TARS to a world as a static model.

## Worlds
Here is a description of the worlds currently stored on this repository:

lit_word.world: An empty world with directional lighting

ball_find_00.world: A large world using a 3D model of terrain from mars

ball_find_01.world: A small world containing several obstacles and a tennis ball path for the rover to follow

## Scripts
Contained in this repository is a python script named "generate_balls.py". This script was created with the purpose of
generating tennis balls at random locations for our worlds. Generate_balls generates n amount of balls in a x by y
square area centered at (0,0) at height z. (n, x, y, and z are user input)

USAGE: Follow the prompt given by the script. It will generate the first part of the XML content in filename (specified
by the user). The second part of the XML content will be generated in filename2. Copy the content from filename, and paste it
right above the line `</state>` in your .world file. Next, copy the content from filename2, and paste it right above `</world>`
in your .world file. Ensure that proper indentation is maintained when transferring the XML to the .world file.
