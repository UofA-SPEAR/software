FROM ros:kinetic-robot

RUN apt-get update && apt-get install -y qt4-default libx264-dev \
                                         ros-kinetic-rqt \
                                         ros-kinetic-gscam \
                                         ros-kinetic-catch-ros \
                                         ros-kinetic-tf \
                                         ros-kinetic-opencv3 \
                                         ros-kinetic-cv-bridge \
                                         gstreamer-0.1 \
                                         libgstreamer0.10-dev \
                                         libgstreamer-plugins-base0.10-dev \
                                         ros-kinetic-rtabmap-ros \
                                         ros-kinetic-move-base \
                                         gstreamer0.10-plugins-good \
                                         python-pip

SHELL ["/ros_entrypoint.sh", "bash", "-c"]
