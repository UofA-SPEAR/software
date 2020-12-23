FROM ros:melodic-robot

RUN apt-get update && apt-get install -y qt4-default libx264-dev \
                                         ros-melodic-rqt \
                                         ros-melodic-gscam \
                                         ros-melodic-catch-ros \
                                         ros-melodic-tf \
                                         ros-melodic-cv-bridge \
                                         ros-melodic-rtabmap-ros \
                                         ros-melodic-move-base \
                                         python-pip \
                                         tmux \
                                         curl \
                                         libxml2-utils \
                                         python-catkin-tools \
                                         iproute2

RUN python -m pip install catkin_lint || echo "Error installing catkin_lint -- this is not a problem on the tx2"
RUN python -m pip install typing
# Later versions of ikpy drop Python 2 support
RUN python -m pip install pygame ikpy==3.0.1

# Install uavcan_gui_tool for can debugging / monitoring
# (see https://uavcan.org/GUI_Tool/Overview/ for install docs)
RUN apt-get update && apt-get install -y python3-pip \
                                         python3-setuptools \
                                         python3-wheel \
                                         python3-numpy \
                                         python3-pyqt5 \
                                         python3-pyqt5.qtsvg \
                                         git-core
RUN pip3 install git+https://github.com/UAVCAN/gui_tool@master

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

RUN ( cd software && bash ./unpack.sh dev )
WORKDIR /software
