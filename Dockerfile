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
                                         curl
RUN apt-get install -y libxml2-utils
RUN apt-get install -y python-catkin-tools
RUN python -m pip install catkin_lint || echo "Error installing catkin_lint -- this is not a problem on the tx2"

SHELL ["/ros_entrypoint.sh", "bash", "-c"]

# On the tx2 we also need:
# sudo apt-get install libusb-0.1-4

COPY . /software

RUN ( cd software && bash ./unpack.sh dev )
