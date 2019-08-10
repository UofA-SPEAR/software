# Must apt install ros-kinetic-rqt-ez-publisher
# roscore must be running
# a node must be subscribed to the topic you're trying to publish to
rosrun rqt_ez_publisher rqt_ez_publisher --slider-file config/arm_new.yaml
