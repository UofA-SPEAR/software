#!/usr/bin/env python2

import uavcan

def response_callback(event):
    # do nothing

msg = uavcan.protocol.RestartNode(magic_number=uavcan.protocol.RestartNode.MAGIC_NUMBER)


node = uavcan.make_node("can0", node_id=120)

node.request(uavcan.protocol.RestartNode.request(), 20, response_callback)
