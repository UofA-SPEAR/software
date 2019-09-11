FROM ros:kinetic-robot

COPY apt-deps /build/apt-deps
RUN apt-get update && apt-get install -y $(cat /build/apt-deps)

RUN mkdir -p ~/ros/src
