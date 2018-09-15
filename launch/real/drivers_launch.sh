#!/bin/bash

num_spheros=$1
filename="$2"

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
	ROS_NAMESPACE="sphero_$i" rosrun sphero_sprk sphero_node.py _address:="${linearray[0]}" &
	((i++))
done
echo "DONE"

cat