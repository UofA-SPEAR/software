[![Build Status](https://travis-ci.com/UofA-SPEAR/software.svg?branch=master)](https://travis-ci.com/UofA-SPEAR/software)

# Software

This is the repository for all our Linux-based software.
There are four ROS packages: spear_rover, spear_station, spear_msgs, and spear_simulator.
Each of these packages fulfills a specific role.

Usage guidelines and more detailed descriptions can be found in the READMEs of each respective package.

## spear_msgs

This is the package that contains all of the ROS messages defined by us, and for use between different packages.
This is a seperate package so that a node or package can depend on it without needing to build other packages.

## spear_rover

This package is for all of the code that runs the rover.
Essentially, any software that is supposed to run on the rover or is hardware-dependant (i.e. drive management or arm kinematics) should be placed in this package.
This includes things which can run on the simulated rover, unless they are meant to exclusively run in simulation.

## spear_simulator

This package contains the code and configuration for anything that runs exclusively on the simulator.
However, the simulator is still generally run by launching something from `spear_rover` (see [the Wiki](https://github.com/UofA-SPEAR/software/wiki/Launch-file-structure)).

## spear_station

This package contains all of the software that will be run at the base station during competition.
This includes our command and control interfaces and essentially anything that isn't run on the rover during competition.

# Docker setup and install instructions

The recommended way to run the software in this repo is with Docker.

## Requirements

- Any linux distro
- Docker

For instructions on how to set up Docker inside a linux virtual machine on Mac, see these pages:

- [Install Instructions for Mac](https://github.com/UofA-SPEAR/software/wiki/Install-Instructions-Mac)

Some distributions have old versions of docker. The recommended way to install
is by using the convenience script, provided by docker.

To do so, run:

``` bash
sudo curl -sSL https://get.docker.com/ | sh
```

Note: you may need to install curl if you don't have it already.

On Ubuntu: `sudo apt-get install curl`

Now install docker-compose. Following the instructions [here](https://github.com/docker/compose/releases), run

    curl -L https://github.com/docker/compose/releases/download/1.25.1-rc1/docker-compose-`uname -s`-`uname -m` -o /usr/local/bin/docker-compose
    chmod +x /usr/local/bin/docker-compose

## Docker setup

If you've never used docker, you'll have to do a little bit of setup.

Create a new group called "docker":

``` bash
sudo groupadd docker
```

Add yourself to that group:

``` bash
sudo usermod -aG docker $USER
```

Log out and log back in for the change to take effect.

Once you have been added to the docker group you should be able to run docker containers as your user (but you will probably still need to run the docker daemon with sudo).

To test your setup, start the docker daemon in its own terminal (if it has not already been started):

``` bash
sudo dockerd
```

Note: if you get a message like the following:

```
INFO[2019-10-09T18:07:23.627014728-06:00] Starting up
failed to start daemon: pid file found, ensure docker is not running or delete /var/run/docker.pid
```

This means that the docker daemon has already been started in the background.
You do not have to run `sudo dockerd` yourself.
Simply proceed to the next step.

Now install and run the test docker image in a new terminal:

``` bash
sudo docker pull hello-world
```

``` bash
docker run hello-world
```

You should see a welcome message from docker. If this works, your docker installation is ready to go!

More information about linux configuration of docker can be found here:
https://docs.docker.com/install/linux/linux-postinstall/

## Build Docker Image

Navigate to the directory containing this readme, and run

    docker-compose build

This will build a docker image from our `Dockerfile` and tag it as `spear`.

## Run Docker container interactively

Navigate to the directory containing this readme, and run

    docker-compose run spear

which runs the `spear` image as a container named `spear-container` (it also runs the `spear-env` image as a container, but that container exits immediately).

### Run with CANBus support

We use CANBus for communicating between boards. Docker requires that some kernel
modules are loaded by the host for CANBus to work. We use the
[`docker-with-can.bash`](./docker-with-can.bash) script to handle that. It loads
the kernel modules and runs `docker-compose run spear` for you.

If you want to use a physical, real CANBus network, i.e. on the rover itself, run:

```bash
./docker-with-can.bash
```

If you want to test CANBus locally (i.e. on your own machine) with a virtual
CANBus network, run:

```bash
./docker-with-can.bash --vcan
```

## Working in Docker

You should be able to edit files in your host machine and build and run the code in the docker container.

To build in the docker container:

``` bash
cd ~/ros
catkin build
```

Or, if you also want to generate things like `compile-commands.json` (which gives us autocomplete within vim, emacs, etc...):

```bash
cd /software
./build.bash
```

# Virtual machine setup and install instructions

You can also develop from a virtual machine.
This is the recommended approach if you use Windows and don't want to dual-boot Linux (due to space constraints, for example).

## Requirements

  - Windows 10 (older versions of windows, or even Linux, probably would work with some modification of the setup instructions)

## Instructions

See [the Wiki page](https://github.com/UofA-SPEAR/software/wiki/Install-Instructions-Windows).

# Additional information

## Running "remote" launch files

Current status is: not working but close.

For now, just ssh into the remote and run the main launch file.
Then, on the local machine, run:

```
export ROS_MASTER_URI=http://tegra-ubuntu:11311/
```

Afterward, any nodes you run on your local machine will automatically connect to the remote machine.

See [this post](https://answers.ros.org/question/41446/a-is-not-in-your-ssh-known_hosts-file/).

## Code Formatting

### C++
```
clang-format-3.9 -i -style=Google <filename>
```

### Python
```
yapf -i <filename>
```

Note: we are linting for python 3.5.

### Launch files

You will have to format the code yourself. If you want, you can use `xmllint --format`, but this removes all blank lines in the code which can harm readability. To check XML formatting for the launch files, run `./xml_lint.bash`. See [the Wiki](https://github.com/UofA-SPEAR/software/wiki/XML-Formatting-and-Linting).
