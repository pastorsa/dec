#!/bin/bash

echo "Node id must start at 1"
echo "Net must be either 0 or 1"

echo "Setting node id to $1 and net to $2"

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
