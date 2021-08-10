#!/usr/bin/env bash

rosbag record \
    /rosout \
    /rosout_agg \
    /tf \
    /tf_static \
    /joint_states \
    /drive_controller/cmd_vel \
    /arm_controller/command \
    /arm_controller/state \
    /feedback/camera_depth/image/compressed \
    /feedback/camera_depth/camera_info \
    /gps/fix \
    /network/receiver_stats \
    /network/sender_stats

