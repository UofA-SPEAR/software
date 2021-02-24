#!/usr/bin/env python2

import uavcan


def response_callback(event):
    # do nothing
    print("Received Callback!")


node = uavcan.make_node("can0", node_id=120)

node.request(uavcan.protocol.RestartNode.Request(), 20, response_callback)

node.spin()
