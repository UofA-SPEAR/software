# Based off of various samples I've found, and the base Ubuntu ros Dockerfiles
# found at https://github.com/osrf/docker_images/blob/master/ros/melodic/ubuntu/

FROM ubuntu:bionic

#########################
# Environment Variables #
#########################

ENV ROS_PYTHON_VERSION 3
ENV ROS_DISTRO melodic
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

#############
# ros setup #
#############

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
	ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
	apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# Install some miscellaneous build dependencies
RUN apt-get update && \
	apt-get install -q -y python3 python3-dev python3-pip build-essential dirmngr gnupg2 curl 

# Add ROS package archive keys, setup sources.list
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list
RUN apt-get update

# Install Python 3 version of bootstrap tools
RUN apt-get install -q -y python3-catkin-pkg python3-rosdep python3-rospkg
RUN pip3 install rosinstall_generator wstool vcstools catkin_tools

# Bootstrap rosdep
RUN rosdep init && rosdep update

############################
# Building ros from source #
############################

WORKDIR /ros_catkin_ws

# Generate list of source repos to download
RUN rosinstall_generator ros_comm --rosdistro "$ROS_DISTRO" --deps --wet-only --tar > "${ROS_DISTRO}-ros_comm-wet.rosinstall"

# Download source repos to ./src
RUN wstool init -j8 src ${ROS_DISTRO}-ros_comm-wet.rosinstall

# Install all source repos. Should build with Python 3 since we set ROS_PYTHON_VERSION=3
RUN rosdep install --from-paths src --ignore-src --rosdistro "$ROS_DISTRO" -y

# Upgrade pip
RUN apt-get install -y python3-pip
RUN python3 -m pip install --upgrade pip

# Install more miscellaneous ros packages
RUN pip3 install catkin_pkg pyyaml empy

# Build and install everything
RUN ./src/catkin/bin/catkin_make_isolated --install -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

# Add our own version of the stock "ros_entrypoint.sh" script to the root directory.
# TODO: Needs to be moved into it's own file and COPY'ed in at some point.
RUN printf '#!/bin/bash\nset -e\n\n# setup ros environment\nsource "/ros_catkin_ws/install_isolated/setup.bash"\nexec "$@"\n' > ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
