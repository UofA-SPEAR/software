# Hardware Interface for drive and arm #

This is a simple node for providing an interface between the higher level,
more abstract control methods from the control systems and the more direct requirements
of the actual hardware.

As of right now it simply splits left and right wheel patterns to all 4 wheels.
It will eventually translate angles to what the arm controllers need at some point.

## Topics

This will output on `/hw_interface/drive` for sending out wheel commands,
as well as `/hw_interface/arm` for sending out arm commands.

## Modifying

To change the wheel or joint numbers, simply edit `include/hardware_interface/mappings.h`.
To change how the mappings work or add new mappings, edit `src/interface.cpp`.

## TODO

- Add nimbro networking stuff
- Add arm translations
