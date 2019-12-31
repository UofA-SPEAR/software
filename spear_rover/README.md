# SPEAR Rover #

This is the rover package. Anything on the rover's computers will be in this package.
If it depends on the hardware configuration of the rover (and isn't part of the simulator), it should be here.

## Launch Files

The main two launch files are `drive.launch` and `navigate.launch`.
Each takes a `simulation` argument, which starts them in simulation if set to true.
Generally, the only reason this would be set to false is when running on the actual rover.
Here's a list of what each launch file does (for an overview of the principles of the launch file structure, see [the Wiki](https://github.com/UofA-SPEAR/software/wiki/Launch-file-structure)).

- `drive.launch` launches all nodes necessary to drive the rover manually. In simulation, this starts `rqt_robot_steering` to control it. Out of simulation, this listens for twist messages from the station.
- `move_base.launch` launches the ROS navigation stack for autonomous navigation (this basically takes sensors (and/or obstacles) and localization, and produces twist messages which must be translated to the motors).
- `navigate.launch` launches all nodes necessary for autonomous navigation. This includes the nav stack, sensors, localization, etc. Navigation is then accomplished by sending commands to `move_base`.
- `robot_description.launch` reads the robot model file and publishes the appropriate tf transforms. This allows us to know e.g. where the camera is in and out of simulation.
- `rover.launch` launches everything needed to drive around the rover by directly sending twist messages to the correct topic. These messages might come from `move_base`, `rqt_robot_steering`, or from the base station.
- `rtabmap_obstacles.launch` downsamples the point cloud from the camera and performs obstacle segmentation using RTAB-Map. Technically RTAB-Map should be doing this internally anyway, but for whatever reason this approach is needed to ensure the obstacle cloud is published fast enough.
- `rtabmap_slam.launch` performs mapping using RTAB-Map.
- `sensors.launch` launches everything needed for all sensors, such as the camera or IMU. Basically it corresponds to anything started via plugins in the simulator.
- `state_estimate.launch` performs localization using EKF's.
- `udp.launch` uses the `nimbro_topic_transport` package to send and receive topics to and from the base station. 
- `visual_odom_and_slam.launch` launches visual odometry using the ZED camera and SLAM using rtabmap.

## Nodes

### ball_detector_node

Pulls from a standard sensor_msgs/Image topic, and finds all yellow ball-shaped objects.
It then publishes these using the BallCoords message type.

#### Parameters

- image_topic: sets the topic name to pull images from.
- output_topic: sets the topic name to publish to.

### ball_drawer_node

Pulls messages from the ball_detector_node and draws the appropriate circles onto an image topic.

#### Parameters

- <node_name>/image_sub_topic: image topic to pull raw images from
- <node_name>/coords_topic: topic to pull ball coordinates from
- <node_name>/image_pub_topic: topic to publish drawn-over images to

### mapper.py

ROS <-> UAVCAN mapper node.
Subscribes to various ROS topics and maps them in a unified interface to the canros node,
which publishes over CAN bus. And vice versa.

### hardware_interface_node

Node to interface with hardware on the rover.

As of now it only maps left/right drive commands to commands to send to the wheels.

### arm_ik_node

Node to take positions for the arm and publish them as angles.

(maybe should be merged into hardware_interface_node?)
