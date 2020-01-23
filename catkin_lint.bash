#!/usr/bin/env bash

python -m catkin_lint ~/ros/src \
  --skip-pkg canros \
  --skip-pkg nimbro_topic_transport \
  --skip-pkg nimbro_service_transport \
  --skip-pkg nimbro_log_transport \
  --skip-pkg rviz_satellite \
  --skip-pkg nimbro_cam_transport \
  --skip-pkg tf_throttle \
  --skip-pkg flexbe_app \
  -W2 \
  "$@"
