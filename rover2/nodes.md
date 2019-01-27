# ball_detector_node #

Pulls from a standard image sensor_msgs/Image topic, and finds all yellow ball-shaped objects.
Publishes these using BallCoords message type.

## Parameters ##

- image_topic: sets the topic name to pull images from
- output_topic: sets the topic name to publish to
