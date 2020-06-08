![](https://github.com/UofA-SPEAR/software/blob/master/doc/launch-files.gv.svg)

The launch files in `spear_rover` and `spear_simulator` are organized into a tree-like hierarchy.
At the top, `spear_rover/drive.launch` launches everything which needs to run on the rover to drive it around manually, and `spear_rover/navigate.launch` launches everything which needs to run on the rover for autonomous navigation.
Both launch files take an argument `simulation` which controls whether they start with the simulator or with the real robot (so to start `navigate.launch` in simulation you could run `roslaunch spear_rover navigate.launch simulation:=true`).

As a general principle, a non-top-level launch file that runs on the robot (simulation or otherwise) should either

- work identically or similarly in or out of simulation, and be in the `spear_rover` package (e.g. `state_estimate.launch`)
- work differently in or out of simulation (but present a similar interface), and have two versions: one in `spear_rover` that runs in the real world, and one in `spear_simulator` which runs in simulation (e.g. `rover.launch`)
- only run in simulation, and be in the `spear_simulator` package (e.g. `steering.launch`)
- only run in the real world, and be in the `spear_rover` package (e.g. `udp.launch`)

The last two usages should be minimized outside of includes to other non-top-level launch files.
The idea is to always have a consistent interface whether in or out of simulation.
