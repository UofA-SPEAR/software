# Artificial intelligence simulator
For training our Mars rover, we have decided to create a simulator that allows us to drive around a 3D model of our rover in a virtual world. This virtual world is designed to be similar to the types of environments we may encounter in competition.

## Getting started
To run our simulator, several dependencies will need to be installed, mainly ROS and Gazebo. We recommend additionally setting up a virtual machine to run our simulator, as it is quite invasive to install on your system and requires that you use Ubuntu Xenial (16.04 LTS). Feel free to use either VirtualBox or VMWare.

Currently our team uses ROS Kinetic Kame, which can be found at http://wiki.ros.org/kinetic/Installation. We recommend the “Desktop-Full Install” ROS package for best compatibility.

Install Gazebo 7, which we use as our physics engine and 3D simulator: http://gazebosim.org/download. Feel free to experiment with creating 3D models and
environments to get a feel for the program.

## 3D models
For the most part, the files stored in this repository will be 3D models used by Gazebo. Copying them to `~/.gazebo/models` will allow you to add them to a simulation using Gazebo's insert menu.

There are so far three main models categories being worked on:
1. The Mars rover. This model will be able to drive and will additionally have sensors attached to it that allow it to observe the simulated world around it, similar to how our physical rover will receive input.
2. The simulated world. Currently, we plan to make the world similar to what the surface of Mars looks like, with an emphasis on creating cliffs, walls, and other obstacles for our rover to navigate.
3. Waypoints. In order for the rover to drive itself, waypoints are required to direct around the map and to the goal. Right now we use tennis balls as waypoints, as they are easy for us to acquire in real life and simplify our image recognition code.

## Worlds
Completed worlds will also be stored in this repository. To run a world in gazebo use: `gazebo <relative_path_to_file>`.
Here is a description of the worlds currently stored on this repository:

lit_word.world: An empty world with directional lighting

ball_find_00.world: A large world using a 3D model of terrain from mars

ball_find_01.world: A small world containing several obstacles and a tennis ball path for the rover to follow
