#!/bin/bash

number_of_nodes=$(rosparam get /num_of_robots)
filename="$1" # config file with initial velocities for each Sphero
robot_name="$2_" # robot name


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
	namespace=$robot_name$i
	ROS_NAMESPACE="$namespace" rosrun sphero_formation reynolds_controller.py _init_vel_x:=${params[1]} _init_vel_y:=${params[2]} &
	((i++))
done


echo "DONE"

cat