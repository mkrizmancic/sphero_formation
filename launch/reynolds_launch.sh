#!/bin/bash

robot_name="$(rosparam get /robot_name)_"
number_of_nodes=$(rosparam get /num_of_robots)
debug="$(rosparam get /debug_boids)"
filename="$1" # config file with initial velocities for each Sphero


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

if [ "$debug" = "true" ]; then
	echo "Calling service to change verbosity level for sphero_0 ReynoldsController to DEBUG..."
	service_name="/${robot_name}0/ReynoldsController/set_logger_level"
	rosservice call --wait ${service_name} "logger: 'rosout' 
level: 'debug'"
fi

cat
