#!/bin/bash

number_of_nodes=$1

echo "Launching $number_of_nodes Kalman filter nodes..."

trap 'killall' INT

killall() {
    trap '' INT TERM     # ignore INT and TERM while shutting down
    echo "**** Shutting down... ****"     # added double quotes
    kill -TERM 0         # fixed order, send TERM not INT
    wait
    echo DONE
}

for ((i=0; i<$number_of_nodes; i++));
do
	ROS_NAMESPACE="sphero_$i" rosrun sphero_formation kalman_filter_node.py &
done
echo "DONE"

cat