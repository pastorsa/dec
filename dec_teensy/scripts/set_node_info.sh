#!/bin/bash

# fuerte
source ${ROS_ROOT}/../rosbash/rosbash
roscd dec_teensy/dec_teensy

FILENAME=node_info.h

echo "#ifndef NODE_INDE_H_" > $FILENAME
echo "#define NODE_INDE_H_" >> $FILENAME

echo "#define NODE_ID $1" >> $FILENAME
echo "#define NODE_ID_HEX 0x0$1" >> $FILENAME

echo "#define NODE_NET $2" >> $FILENAME
echo "#define NODE_NET_HEX 0x0$2" >> $FILENAME

echo "#endif" >> $FILENAME
