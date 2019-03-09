# SPEAR Rover #

This is the rover package. Anything on the rover's computers will be in this package.
If it depends on the hardware configuration of the rover (and isn't part of the simulator),
it should be here.

## Nodes ##

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
