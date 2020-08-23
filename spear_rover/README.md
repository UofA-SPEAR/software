# SPEAR Rover #

This is the rover package. Anything on the rover's computers will be in this package.
If it depends on the hardware configuration of the rover (and isn't part of the simulator), it should be here.

## Launch Files

The main two launch files are `drive.launch` and `navigate.launch`.
Each takes a `simulation` argument, which starts them in simulation if set to true (the default), and a `station` argument, which can be set to the IP address of another computer which is running the station nodes (set to localhost by default).

- `drive.launch` launches all rover nodes needed for manual driving. You can optionally point this at another computer to use as the station.
- `navigate.launch` launches all nodes necessary for autonomous navigation. This includes the nav stack, sensors, localization, etc. Navigation is then accomplished by sending commands to `move_base`. Otherwise functions like `drive.launch`

For an overview of the principles of the launch file structure, see [the Wiki](https://github.com/UofA-SPEAR/software/wiki/Launch-file-structure).
For more information on what each launch file does, see the comments in each launch file.

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
