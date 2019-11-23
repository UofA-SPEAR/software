# This directory is for mounting the ros includes in a way that autocomletion
# engines can get to them.
mkdir -p $(pwd)/.docker-opt-ros-mount

# see: http://fabiorehm.com/blog/2014/09/11/running-gui-apps-with-docker/
docker run \
    -it \
    -e DISPLAY \
    -e ROS_IP=127.0.0.1 \
    -e QT_X11_NO_MITSHM=1 \
    -e DOCKER_HOST_REPO_LOCATION=$(pwd) \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTHORITY:/root/.Xauthority  \
    -v $(pwd):/software  \
    -v $(pwd)/.docker-opt-ros-mount:/tmp/docker-opt-ros-mount \
    -v $(pwd)/.docker-root-ros-include-mount:/tmp/docker-root-ros-include-mount \
    --device=/dev/dri:/dev/dri \
    --net=host \
    spear bash -c 'echo "Copying ros headers from container /opt/ros to host-accessible location (for autocompletion)..." && mkdir -p /tmp/docker-opt-ros-mount && cp -r /opt/ros/* /tmp/docker-opt-ros-mount/ && echo "Doing the same with /root/ros/devel/include..." && mkdir -p /tmp/docker-root-ros-include-mount && cp -r /root/ros/devel/include/* /tmp/docker-root-ros-include-mount/ && echo -e "Success!\nRun the build.bash script to re-copy headers."; /ros_entrypoint.sh bash -l'
