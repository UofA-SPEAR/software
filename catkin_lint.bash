#!/usr/bin/env bash

python -m catkin_lint ~/ros/src \
  --skip-pkg case_moveit_config \
  --skip-pkg spear_flexbe_states \
  --skip-pkg canros \
  --skip-pkg nimbro_topic_transport \
  --skip-pkg nimbro_service_transport \
  --skip-pkg nimbro_log_transport \
  --skip-pkg rviz_satellite \
  --skip-pkg nimbro_cam_transport \
  --skip-pkg tf_throttle \
  --skip-pkg flexbe_app \
  --skip-pkg ros_numpy \
  --skip-pkg roboticsgroup_upatras_gazebo_plugins \
  -W2 \
  "$@"
