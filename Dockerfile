FROM ros:noetic-robot
ARG DEBIAN_FRONTEND=noninteractive

# nimbro_network needs qt4
RUN apt-get update && apt-get install -y software-properties-common
RUN add-apt-repository ppa:rock-core/qt4

# Rosdep doesn't work for hector-gazebo-plugins ??
RUN apt-get update && apt-get install -y libx264-dev \
                                         ros-noetic-rqt \
                                         ros-noetic-catch-ros \
                                         ros-noetic-tf \
                                         ros-noetic-cv-bridge \
                                         ros-noetic-rtabmap-ros \
                                         ros-noetic-move-base \
                                         ros-noetic-hector-gazebo-plugins \
                                         python3-pip \
                                         git \
                                         tmux \
                                         curl \
                                         libxml2-utils \
                                         python3-catkin-tools \
                                         iproute2

RUN pip3 install catkin_lint || echo "Error installing catkin_lint -- this is not a problem on the tx2"
RUN pip3 install pygame ikpy transforms3d osrf-pycommon

# Install nunavut for serialization code generation
RUN pip3 install nunavut

SHELL ["/ros_entrypoint.sh", "/bin/bash", "-c"]

# On the tx2 we also need:
# sudo apt-get install libusb-0.1-4

# Add a nice default .tmuxrc
WORKDIR /root
RUN git clone https://github.com/gpakosz/.tmux.git
RUN ln -s -f .tmux/.tmux.conf
RUN cp .tmux/.tmux.conf.local .
RUN sed -i '/#set -g mouse on/c\set -g mouse on' .tmux.conf.local
WORKDIR /

# nvidia-container-runtime
ENV NVIDIA_VISIBLE_DEVICES \
    ${NVIDIA_VISIBLE_DEVICES:-all}
ENV NVIDIA_DRIVER_CAPABILITIES \
    ${NVIDIA_DRIVER_CAPABILITIES:+$NVIDIA_DRIVER_CAPABILITIES,}graphics

# Download gazebo model database
RUN git clone https://github.com/osrf/gazebo_models ~/.gazebo/models


COPY . /software

# Set IS_DOCKER to true so setup-vcan.bash and setup-can.bash don't try
# modprobing things
ENV IS_DOCKER true

RUN ( cd software && make unpack )
WORKDIR /software
