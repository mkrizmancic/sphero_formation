#!/bin/bash

num_spheros=$1
filename="$2"
data_stream="$3"

echo "Launching $num_spheros Sphero driver nodes..."

trap 'killall' INT

killall() {
    trap '' INT TERM     # ignore INT and TERM while shutting down
    echo "**** Shutting down... ****"     # added double quotes
    kill -TERM 0         # fixed order, send TERM not INT
    wait
    echo DONE
}

i=0
head -$num_spheros $filename |
while read line; do
	linearray=($line)
	ROS_NAMESPACE="sphero_$i" rosrun sphero_sprk_ros sphero_node.py _address:="${linearray[0]}" _data_stream:=$data_stream &
	((i++))
done
echo "DONE"

cat