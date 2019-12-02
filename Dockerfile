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
                                         python-pip \
                                         tmux \
                                         vim \
                                         nano
RUN apt-get install -y libxml2-utils
RUN apt-get install -y python-catkin-tools
RUN python -m pip install catkin_lint

SHELL ["/ros_entrypoint.sh", "bash", "-c"]

COPY . /software

RUN apt-get update
RUN ( cd software && bash ./unpack.sh dev )
