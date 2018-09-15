#!/bin/bash

number_of_nodes=$1
filename="$2" # config file with initial velocities for each Sphero

echo "Launching $number_of_nodes Reynolds controller nodes..."

trap 'killall' INT

killall() {
    trap '' INT TERM     # ignore INT and TERM while shutting down
    echo "**** Shutting down... ****"     # added double quotes
    kill -TERM 0         # fixed order, send TERM not INT
    wait
    echo DONE
}

i=0
head -$number_of_nodes $filename |
while read line; do
	params=($line)
	ROS_NAMESPACE="sphero_$i" rosrun sphero_formation reynolds_controller.py _init_vel_x:=${params[1]} _init_vel_y:=${params[2]} &
	((i++))
done


echo "DONE"

cat