FROM ros:kinetic-robot

RUN apt-get update && apt-get install -y ros-kinetic-desktop-full qt4-default libx264-dev

SHELL ["/ros_entrypoint.sh", "bash", "-c"]
