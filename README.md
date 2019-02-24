[![Build Status](https://travis-ci.org/UofA-SPEAR/rover2.svg?branch=master)](https://travis-ci.org/UofA-SPEAR/rover2)
# Rover 2
This is the rover package. Anything on the rover's computers will be in this package.

NOTE: this branch depends on arm\_position.msg from the station2 repo branch arm\_controls\_interface\_lyndon\_joy

## Dependencies
```bash
sudo apt update
sudo apt install python-pip
```

## Building
Run `build.bash` from any directory to build everything. This also generates
a `compile_commands.json` file and symlinks it to this repo's root to allow for
things like code completion in c++ files.

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
