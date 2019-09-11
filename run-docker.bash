# see: http://fabiorehm.com/blog/2014/09/11/running-gui-apps-with-docker/
docker run \
    -it \
    -e DISPLAY \
    -e ROS_IP=127.0.0.1 \
    -e QT_X11_NO_MITSHM=1 \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $XAUTHORITY:/root/.Xauthority  \
    --net=host \
    spear
