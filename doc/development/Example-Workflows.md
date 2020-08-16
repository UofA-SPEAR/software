# Example Workflows

Here are some example editor configurations and workflows.
You do not have to set things up exactly like this, but it may be useful to compare these to your own workflow.

## Development with VSCode, in docker

### Setup

Clone the _software_ repo somewhere (I used `~/source/spear/software`).
Check the readme to make sure you have all the necessary dependencies installed.
Now, in the main repo directory, run

```bash
docker-compose build
```

to build our docker image, and

```bash
docker-compose run spear
```

to run the image as a container.

Now open up vscode and install the `Remote-Containers` extension.
Run the command `Remote-Containers: Attach to running container` to attach to the spear container created earlier.
Now open the folder `/software` in the container and start editing.
Any changes you make will be immediately reflected in the `software` folder outside the container, and vice versa.

### C++ autocomplete

Now we need to get C++ autocomplete working.
Install the `clangd` extension (it's _much_ better than the default C++ extension), and install the clangd language server when it prompts you to do so.
`clangd` knows what things like include paths are by reading a file called `compile-commands.json`.
Our `build.bash` script automatically copies over this file, but currently the method for doing so is broken in this workflow.
To fix it, go to `build.bash` and replace this line

```
cp ~/ros/build/compile_commands.json ${SCRIPT_DIR}/compile_commands.original.json
```

with this one

```
cp ~/ros/build/compile_commands.json ${SCRIPT_DIR}/compile_commands.json
```

and delete everything after that line.

Now run `./build.bash` and C++ autocomplete should work.

### Python autocomplete

Python autocomplete is a similar process.
Install the `Python` and `Pylance` extensions (pylance is important because it supports type hinting comments in Python 2).
Go to your workspace settings and change the python language server to pylance, and change `python.analysis.typeCheckingMode` to _basic_ (the default is _none_).
Open a python file and make sure you have `Python 2.7.17 64-bit` (or similar) selected as the python to use in the project.

Python doesn't have an equivalent of `compile-commands.json`, so you need to set some paths yourself.
Add the following to `python.analysis.extraPaths`:

- _/opt/ros/melodic/lib/python2.7/dist-packages_ (system ros packages)
- _/root/ros/devel/lib/python2.7/dist-packages_ (ros packages installed from source)
- _/software/spear_rover/python_ (probably if we had things configured properly this wouldn't be needed)

(the first two of these are also in PYTHONPATH, but for some reason vscode doesn't seem to recognize that like you'd expect)

### Code formatting

Before you commit your code, you should format it.
For C++, the clangd extension should handle this for you.
Just run `Format Document` and select the extension as the formatting provider if necessary.
For Python, run `Format Document` and select `yapf` as the formatting provider.

### Building and running

After changing a C++ source file, you will need to run `./build.bash` for the changes to be reflected in anything you run.
Changes to Python files will be reflected immediately after changing them.

You can use the `rosrun` and `roslaunch` commands from anywhere.
You don't have to be in the `/software` directory.

The container should support communications with the rover out of the box, as long as you've configured your network settings per the rover communications guide.
Likewise, support for peripherals such as joysticks and cameras should just work.

