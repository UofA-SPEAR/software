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

### C++ and Python autocomplete

Now we're going to get autocomplete and code formatting working.
Install the following extensions

  - cland (`llvm-vs-code-extensions.vscode-clangd`)
  - Python (`ms-python.python`)
  - Pylance (`ms-python.vscode-pylance`)

And add the following to your workspace settings:

```json
{
    "files.watcherExclude": {
        "**/.docker-*/**": true
    },
    
    "clangd.path": "clangd-9",
    "[cpp]": {
        "editor.defaultFormatter": "llvm-vs-code-extensions.vscode-clangd",
        "editor.formatOnSave": true,
        "editor.formatOnType": true
    },
    
    "python.languageServer": "Pylance",
    "python.pythonPath": "/usr/bin/python",
    "python.analysis.extraPaths": [
        "/home/nvidia/software/spear_rover/python",
        "/home/nvidia/ros/devel/lib/python2.7/dist-packages",
        "/opt/ros/melodic/lib/python2.7/dist-packages"
    ],
    "python.analysis.typeCheckingMode": "basic",
    "python.formatting.provider": "yapf",
    "[python]": {
        "editor.formatOnSave": true
    },
}
```

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

### Building and running

After changing a C++ source file, you will need to run `./build.bash` for the changes to be reflected in anything you run.
Changes to Python files will be reflected immediately after changing them.

You can use the `rosrun` and `roslaunch` commands from anywhere.
You don't have to be in the `/software` directory.

The container should support communications with the rover out of the box, as long as you've configured your network settings per the rover communications guide.
Likewise, support for peripherals such as joysticks and cameras should just work.

### Saving changes and git

Changes to the `/software` directory within the docker container are mirrored outside of it, so you can stop and restart the image and your changes will still be saved.
But since commands are 
