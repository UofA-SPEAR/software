FROM ros:kinetic-robot

RUN ./ros_entrypoint.sh bash -c 'mkdir -p ~/ros/src && (cd ~/ros; /opt/ros/kinetic/bin/catkin_make)'

# deps
COPY docker/apt-deps /build/apt-deps
RUN bash -c 'apt-get update && apt-get install -y $(cat /build/apt-deps)'

COPY docker/git-deps /build/git-deps
RUN bash -c '/build/git-deps'

# SPEAR packages
COPY spear_msgs /root/ros/src/spear_msgs
COPY spear_rover /root/ros/src/spear_rover

RUN ./ros_entrypoint.sh bash -c '(cd ~/ros; /opt/ros/kinetic/bin/catkin_make --force)'
RUN rosdep install --from-paths ~/ros/src --ignore-src -r -y
