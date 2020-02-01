# SPEAR Simulator
For testing our Mars rover, we have decided to use a simulator that allows us to drive around a 3D model of our rover in a virtual world. This virtual world is designed to be similar to the types of environments we may encounter in competition.
The `spear_simulator` ROS package contains all simulator worlds and models, as well as all code which runs exclusively in the simulator.

## Launch Files

To launch the simulator, run one of the top-level launch files in the `spear_rover` package, passing in `simulation:=true`.
The launch files in this package are generally called by others within `spear_rover`, and either correspond to launch files in `spear_rover` or start activities only applicable in simulation.

- `rover.launch` launches everything needed to drive the rover around by sending Twist messages to the appropriate topic. Specifically, it 
  - Starts gazebo with the simulated world
  - Puts the robot model within the world
  - Launches the `controller_manager` to translate Twist messages to wheel commands
  - Launches `robot_description.launch`

- `steering.launch` launches the rqt steering GUI which sends Twist messages to move the robot around.

- `visual_odom.launch` launch visual odometry using RTAB-Map.

## 3D models

NOTE: Our models are located in the `spear_simulator/models` directory.
The `unpack.sh` script adds this to the `GAZEBO_MODEL_PATH` environment variable.
This needs to be done since by default, Gazebo only looks in `~/.gazebo` directory for models.

NOTE: ROS kinetic uses Gazebo 7. This version of Gazebo only supports .dae, .stl, and .svg model formats. If you are adding a new model to this repo, it must be in one of the formats specified above. If your 3D model is not in one of the formats above, consider using Blender to convert your file to a .dae (the easiest format to convert to).

This repo contains many object that will help the rover learn. From obstacles to waypoints, objects contained in this repo will provide data for the rover to improve the way it drives. Right now we use tennis balls as waypoints, as they are easy for us to acquire in real life and simplify our image recognition code. There are multiple variation of tennis balls located in this repo to test the accuracy of the rover's ability to recognize tennis balls.

## Rover model
In the models folder of this repo is a TARS rover 3D model. Using roslaunch, a fully functioning virtual version of the rover can be used for our simulations.

If you would like to view a static version of TARS, simply follow the instructions in the 3D models section to add TARS to a world as a static model.

The TARS `urdf` file also loads several gazebo plugins to simulate the robot's sensors, such as the main camera and IMU.

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
