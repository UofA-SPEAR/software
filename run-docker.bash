# see: http://fabiorehm.com/blog/2014/09/11/running-gui-apps-with-docker/
docker run -it -e DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -v $XAUTHORITY:/root/.Xauthority --net=host spear
