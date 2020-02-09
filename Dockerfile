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
                                         vim \
                                         nano \
                                         libxml2-utils \
                                         python-catkin-tools \
                                         iproute2

RUN python -m pip install catkin_lint

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

COPY . /software

# Set IS_DOCKER to true so setup-vcan.bash and setup-can.bash don't try
# modprobing things
ENV IS_DOCKER true

RUN ( cd software && bash ./unpack.sh dev )
