FROM ros:melodic-robot
ARG DEBIAN_FRONTEND=noninteractive

# nimbro_network needs qt4
# RUN apt-get update && apt-get install -y software-properties-common
# RUN add-apt-repository ppa:rock-core/qt4

# Rosdep doesn't work for hector-gazebo-plugins ??
RUN apt-get update && apt-get install -y libx264-dev \
                                         ros-melodic-rqt \
                                         ros-melodic-catch-ros \
                                         ros-melodic-tf \
                                         ros-melodic-cv-bridge \
                                         ros-melodic-rtabmap-ros \
                                         ros-melodic-move-base \
                                         ros-melodic-hector-gazebo-plugins \
                                         python-pip \
                                         git \
                                         tmux \
                                         curl \
                                         libxml2-utils \
                                         python-catkin-tools \
                                         iproute2

RUN pip install catkin_lint || echo "Error installing catkin_lint -- this is not a problem on the tx2"
RUN pip install \
    pygame \
    # ikpy==3.0.1 \
    transforms3d==0.2.1 \
    osrf-pycommon


# Install nunavut for serialization code generation
RUN apt-get update && apt-get install -y python3-pip
RUN pip3 install --upgrade pip
RUN pip3 install nunavut
RUN pip3 install git+https://github.com/UAVCAN/gui_tool@develop

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
