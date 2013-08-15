#!/bin/bash

if [ $# -ne 2 ]
then
  echo "Usage: `basename $0` <net> <node_id>"
  exit -1
fi

if [ "$1" != "0" ] && [ "$1" != "1" ]
then
  echo "ERROR: Net must be either 0 or 1"
  echo "ERROR: Net must be either 0 or 1"
  echo "ERROR: Net must be either 0 or 1"
  echo "ERROR: Net must be either 0 or 1"
  exit -1
fi

if [ "$2" != "1" ] && [ "$2" != "2" ] && [ "$2" != "3" ] && [ "$2" != "4" ] && [ "$2" != "5" ] && [ "$2" != "6" ] && [ "$2" != "7" ] && [ "$2" != "8" ]
then
  echo "ERROR: Node id must be within 1..8"
  echo "ERROR: Node id must be within 1..8"
  echo "ERROR: Node id must be within 1..8"
  echo "ERROR: Node id must be within 1..8"
  exit -1
fi


# fuerte
source ${ROS_ROOT}/../rosbash/rosbash
roscd dec_teensy/dec_teensy

FILENAME=node_info.h

echo "#ifndef NODE_INDE_H_" > $FILENAME
echo "#define NODE_INDE_H_" >> $FILENAME
echo "" >> $FILENAME
echo "// Setting ip to 10.0.$1.$2" >> $FILENAME
echo "" >> $FILENAME
echo "#define NODE_ID $2" >> $FILENAME
echo "#define NODE_ID_HEX 0x0$2" >> $FILENAME
echo "" >> $FILENAME
echo "#define NODE_NET 0x0$1" >> $FILENAME
echo "" >> $FILENAME
echo "#endif" >> $FILENAME

echo "`cat node_info.h`"