# Station
This is the ground control package. It contains all the tools nessesary for
controlling the rover. 

## Launch Files

The main launch file here is `station.launch`.
It takes a `rover` argument which you can point to the IP address of the computer running the rover nodes.
By default, localhost is used (and the nimbro_network communications channel is disabled in favor of normal ROS communication).
