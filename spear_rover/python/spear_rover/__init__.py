"""
This file allows for an entry point to split code, like utility functions,
out of the main node file. It's generally a good idea to import functions from
this module, as other submodules might have a bunch of other internal functions
polluting your global namespace.
"""

from .util import map_ros_to_can, map_can_to_ros, test_bit
