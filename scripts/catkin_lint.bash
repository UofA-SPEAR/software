#!/usr/bin/env bash

python3 -m catkin_lint ~/ros/src \
  --skip-pkg case_moveit_config \
  --skip-pkg spear_flexbe_states \
  --skip-pkg nimbro_topic_transport \
  --skip-pkg nimbro_service_transport \
  --skip-pkg nimbro_log_transport \
  --skip-pkg rviz_satellite \
  --skip-pkg nimbro_cam_transport \
  --skip-pkg tf_throttle \
  --skip-pkg ros_numpy \
  --skip-pkg roboticsgroup_upatras_gazebo_plugins \
  --skip-pkg ar_track_alvar \
  --skip-pkg ar_track_alvar_msgs \
  -W2 \
  "$@"
