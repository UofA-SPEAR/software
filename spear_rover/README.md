# SPEAR Rover #

This is the rover package. Anything on the rover's computers will be in this package.
If it depends on the hardware configuration of the rover (and isn't part of the simulator),
it should be here.

## Launch Files

### drive.launch

This launches all the nodes necessary to drive the rover manually.
This will launch a canros server.

### udp_rover.launch

This launches nodes to send and receive messages using the udp protocol.
We use udp to send commands from the base station to the rover and camera feeds from the rover to the base station.

### zed.launch

This launches the ZED stereoscopic camera on the rover.
It launches the ZED camera's own rgbd odometry node.
It also includes the main `rtabmap.launch` file.

### rtabmap.launch

This is our main file for launching and configuring rtabmap.
It launches an rtabmap mapping node to perform SLAM based on messages from the ZED camera.

### state_estimate.launch

This launches ekf nodes which are used to fuse odometry from various sources on the rover including visual odometry, IMU, GPS, and wheel encoders.

### sensors.launch

Launches sensors including the IMU and GPS.

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
