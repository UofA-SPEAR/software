FROM ros:kinetic-robot

RUN apt-get update && apt-get install -y qt4-default libx264-dev \
                                         ros-kinetic-rqt \
                                         ros-kinetic-gscam \
                                         ros-kinetic-catch-ros \
                                         ros-kinetic-tf

SHELL ["/ros_entrypoint.sh", "bash", "-c"]
