# Autonomy Simulator
For training our Mars rover, we have decided to create a simulator that allows us to drive around a 3D model of our rover in a virtual world. This virtual world is designed to be similar to the types of environments we may encounter in competition.
We have packaged all the models, worlds, and config files in a ros package and the simulator can be run using roslaunch to launch various files.
See below for an explanation of the different launch files.

## Launch Files

Each of these launch files can be launched using the command `roslaunch spear_simulator <name of launch file>`.

- `gazebo.launch`
  - Launches Gazebo.
  - Loads our worlds.
  - Loads the the rover model.

- `diffdrive.launch`
  - Includes `gazebo.launch`. If you launch this file, you don't have to also
    launch `gazebo.launch`
  - Launches nodes that allow us to control the rover.

- `rtabmap.launch`
  - Includes the main `rtabmap.launch` file from the `spear_rover` package.
    The main `rtabmap.launch` file launches the mapping node.
  - Also launches an `rgbd_odometry` node (also part of the `rtabmap` package).
	Note that the zed camera on the rover has its own rgbd odometry node.
	However, in the simulator, we must use the one from `rtabmap`.

- `nav.launch`
  - Launches a `move_base` node and loads all our yaml config files from the
    planners and costmaps.
  - Note: `nav.launch` as well as the files in the `config` directory should
    eventually be moved to the `spear_rover` package since they provide actual
    functionality and are not just for the simulator.

## 3D models

NOTE: Our models are located in the `spear_simulator/models` directory.
The `unpack.sh` script adds this to the `GAZEBO_MODEL_PATH` environment variable.
This needs to be done since by default, Gazebo only looks in `~/.gazebo` directory for models.

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
