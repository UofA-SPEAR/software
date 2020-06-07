# How to Add Dependencies

There are different ways that dependencies should be added depending on whether the dependecies are ROS packages or Ubuntu package installed with apt.

## Temporarily installing a ROS package dependency

Sometimes, you might want to temporarily install a ROS package to test it out before making the dependency permanent.
To do this, simply start a docker container then use `apt-get install` to install the package.

You will need to change the name of the ROS package to match the naming scheme of ROS packages in the apt repository.
The involves replacing all the underscores in the ROS package name with hyphens and add the prefix `ros-kinetic` (or `ros-melodic` if you're using melodic).
For example, if you wanted to install the ROS package `move_base`, you would use this command:

```bash
apt-get install ros-kinetic-move-base
```

Note that packages you install with apt while a docker container is running will *not* remain installed after you stop and start the container again.
If you decide that the ROS package should be installed permanently, follow the instructions below.

## Dependency is a ROS package

If the dependency is a ROS package, it should be added to a `package.xml` for one of our packages.
For example, our package `spear_rover` depends on the ROS `robot_localization` package.
Therefore, we have added the following lines to `spear_rover/package.xml`:

```xml
  <build_depend>robot_localization</build_depend>
  <exec_depend>robot_localization</exec_depend>
```

Depending on whether the dependency is need at compile time, run time, or both, you may need to add it as a `build_depend` and/or an `exec_depend`.
See [this](http://wiki.ros.org/catkin/package.xml) documentation for details about the differences between `build_depend`, `exec_depend`, and other types of ROS dependencies.

## Dependency is not a ROS package but can be installed using apt

In this case, add the appropriate `apt-get install` command to `Dockerfile` as a `RUN` instruction. 
