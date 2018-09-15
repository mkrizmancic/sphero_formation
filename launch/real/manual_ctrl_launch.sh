#!/bin/bash

number_of_nodes=$1
sens=$2
echo "Launching $number_of_nodes Manual control nodes..."

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
	ROS_NAMESPACE="sphero_$i" rosrun sphero_formation man_control.py /joystick_input:=/joy _sensitivity:=$sens &
done
echo "DONE"

cat