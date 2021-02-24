#!/bin/bash


###### Build UAVCAN messages
# Temporary solution until we get python 3
DSDL=/software/spear_rover/uavcan_dsdl
nnvg $DSDL/public_regulated_data_types/uavcan --target-language c --outdir spear_rover/dsdl
nnvg $DSDL/spear -I $DSDL/public_regulated_data_types/uavcan --target-language c --outdir spear_rover/dsdl
