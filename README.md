[![Build Status](https://travis-ci.org/UofA-SPEAR/rover2.svg?branch=master)](https://travis-ci.org/UofA-SPEAR/rover2)
# Rover 2
This is the rover package. Anything on the rover's computers will be in this package.

## Dependencies

### [`web_video_server`](https://github.com/RobotWebTools/web_video_server)

```bash
$ sudo apt update
$ sudo apt install ros-kinetic-web-video-server
```

## Docker

1. Setup the environment. (you only need to do this once)
```bash
docker pull ros:kinetic-robot
docker build --rm -t spear-env - < spear-env.Dockerfile
```

2. build the project.
```bash
docker build --rm -t spear-rover2 .
```

3. get an interactive shell
```bash
docker run --rm -it spear-rover2
```
