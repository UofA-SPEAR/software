# Navigation and localization

## Navigation

We use the [ROS Navigation Stack](https://wiki.ros.org/navigation) for navigation and path planning.
The navigation stack is a complicated and highly configurable piece of software, but in our configuration it takes as input

- An estimate of the rover's current pose from the global EKF, as an [Odometry](https://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html) message from the topic `/ekf/map/odometry/filtered`.
- A point cloud of all obstacles in front of the rover, as found using RTAB-Map's point cloud obstacle segmentation, from the `/obstacles_detection/obstacles` topic. The navigation stack can also take raw sensor inputs and find obstacles from those, but RTAB-Map's method is more sophisticated.

> The navigation stack can also take as input a map of the environment. This is a sort of "base map" to which the navigation stack adds obstacles from its sensors. There is no requirement that this map be similar or identical to the map used in localization or SLAM. It can even be blank, in which case navigation is performed entirely using detected obstacles. We do not use this input, however.

The navigation stack outputs [Twist](https://docs.ros.org/melodic/api/geometry_msgs/html/msg/Twist.html) messages to the topic `/rover_diff_drive_controller/cmd_vel` describing how the rover should move.
It is not responsible for determining how exactly the wheels should turn in order to perform this movement.

### More details about move_base

#### Overview of components

A *costmap* is a map of the robot's environment used by `move_base`.
A costmap is configurable mainly through its set of *layers*, but I don't know much about how that works so I'll skip it for now.
The important thing about a costmap is that it can have various sensors associated with it, which add or remove obstacles from the map.

`move_base` uses two costmaps, which are hard-coded to be called `global_costmap` and `local_costmap`.
The `global_costmap` is used by the *global planner*, which performs high-level path planning, and the `local_costmap` is used by the *local planner*, which generates wheel movements to roughly follow the global plan while taking into account robot dynamics and obstacles.
The global planner and the local planner are sometimes referred to in the source code as the *planner* and the *controller* (unless I'm misreading the source code).

By default, the global planner is `navfn/NavfnROS` and the local planner is `base_local_planner/TrajectoryPlannerROS`.
These can be changed by setting the `base_global_planner` and `base_local_planner` parameters in `move_base`.

#### Configuring and launching the components

The navigation stack is launched by launching the `move_base` node with an enormous amount of parameters in various namespaces.
Suppose that the `move_base` node is launched in the global namespace and named `move_base`.
Then,

- Parameters of `move_base` itself (e.g. `base_local_planner`, `base_global_planner`) go under `/move_base`.
- Parameters of the local costmap (e.g. `global_frame`, `plugins`) go under `/move_base/local_costmap`, and parameters of the global costmap go under `/move_base/global_costmap`.
  Parameters of a specific layer of a costmap go under `/move_base/<costmap>/<layer-name>`.
- Parameters of the global and local planners (e.g. `allow_unknown`, `xy_goal_tolerance`) go into a namespace which depends on what planner is being used.
  For example, if `navfn/NavfnROS` is being used as the global planner, then its parameters go under `/move_base/NavfnROS`.

Here's an example (incomplete) configuration:

```yaml
move_base:
  # Configure move_base node
  base_local_planner: base_local_planner/TrajectoryPlannerROS
  base_global_planner: navfn/NavfnROS
  ...
  # Configure local planner
  TrajectoryPlannerROS:
    xy_goal_tolerance: 1.0
    yaw_goal_tolerance: 0.5
    ...
  # Configure global planner
  NavfnROS:
    allow_unknown: true
    ...
  # Configure local costmap
  local_costmap:
    global_frame: odom
    ...
    # The map uses two layers, which I choose to call "obstacle_layer" and "inflation_layer"
    plugins:
    - {name: obstacle_layer,  type: "costmap_2d::VoxelLayer"}
    - {name: inflation_layer, type: "costmap_2d::InflationLayer"}
    # Configure the layers
    obstacle_layer:
      # The obstacle layer uses a laser scan to add/remove obstacles.
      # I choose to call the sensor "laser_scan_sensor".
      observation_sources: laser_scan_sensor
      laser_scan_sensor:
        data_type: LaserScan
        topic: /obstacles_detection/scan
        ...
    inflation_layer:
      ...
  # Configure global costmap
  global_costmap:
    global_frame: map
    ...
```

In practice, this configuration is usually loaded from multiple yaml files.
See the launch file [move_base.launch](https://github.com/UofA-SPEAR/software/blob/master/spear_rover/launch/move_base.launch) in the `spear_rover` package, as well as the configuration files in [spear_rover/config/move_base](https://github.com/UofA-SPEAR/software/tree/master/spear_rover/config/move_base).

#### List of parameters

Finding out which parameters can be configured for each component is a lot trickier than it should be.
Not all parameters are documented, and `move_base` tries to be smart and move parameters around if necessary, which makes it tricky to read code produced by other groups.
In theory, the following sources list the parameters:

- move_base: https://wiki.ros.org/move_base#Parameters
- Costmaps: https://wiki.ros.org/costmap_2d#costmap_2d.2Flayered.Parameters
- Costmap layers
  - static: https://wiki.ros.org/costmap_2d/hydro/staticmap
  - obstacle: https://wiki.ros.org/costmap_2d/hydro/obstacles
  - inflation: https://wiki.ros.org/costmap_2d/hydro/inflation
- Local planner (default): https://wiki.ros.org/base_local_planner#Parameters
- Global planner (default): https://wiki.ros.org/navfn#Parameters

If a parameter is being used that doesn't show up in the preceding lists, it's probably undocumented and you'll need to go to the source code to figure out what it is.
The code can be found here: https://github.com/ros-planning/navigation.
As a quick note, the code for individual costmap layers is found in the `costmap_2d/plugins` directory.


## Localization

As mentioned above, the ROS navigation stack needs an estimate of the rover's current pose.
We use multiple [EKFs](https://en.wikipedia.org/wiki/Extended_Kalman_filter) implemented by the package [robot_localization](https://docs.ros.org/melodic/api/robot_localization/html/index.html) to provide this estimate.

The first EKF, the *local* or *odom* EKF, provides a smooth but drifting position estimate suitable for navigation.
The second, the *global* or *map* EKF, provides a non-smooth but stable position estimate.
The first EKF fuses

- relative linear velocity from wheel odometry
- linear and angular velocity from visual odometry
- absolute orientation from the IMU

and the second fuses all of these sources, with the addition of

- absolute position from the GPS

For more information, see the launch file [state_estimate.launch](https://github.com/UofA-SPEAR/software/blob/master/spear_rover/launch/state_estimate.launch) in the `spear_rover` package, as well as the configuration files in [spear_rover/config/ekf](https://github.com/UofA-SPEAR/software/tree/master/spear_rover/config/ekf).
