# Mission planner GUI

The mission planner loads commands from a text file and runs them in sequence. The file `example-actions` is an example of a file which could be used.

## Adding new actions

You can add support for new actions by editing `src/make_state.js`. In particular you will need to add a new `make_<something>_state()` function and add it to the map in `make_state()`.

## Building and running the code

The code uses NPM for package management. This needs to be installed using [NVM](https://github.com/nvm-sh/nvm) (the version from apt throws errors if used).

To install dependencies, run the following from the directory containing this Readme:

    sudo apt install ros-kinetic-rosbridge-server
    curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.34.0/install.sh | bash
    nvm install node
    npm install
    npm install -g serve

After running the second command you may have to close and re-open the shell.


The mission planner needs rosbridge to be running in order to interface with ROS. To run rosbridge, use,

    roslaunch rosbridge_server rosbridge_websocket.launch

(I suppose this should be put in a launch file or something)


To run the code for development purposes, use

    npm start

The production version is built using

    npm build

and run with

    serve -s build

