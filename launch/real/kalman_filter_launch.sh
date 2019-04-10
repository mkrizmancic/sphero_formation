#!/bin/bash

number_of_nodes=$(rosparam get /num_of_robots)
debug="$(rosparam get /debug_kalman)"

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

if [ "$debug" = "true" ]; then
	echo "Calling service to change verbosity level for sphero_0 Kalman to DEBUG..."
	service_name="/sphero_0/Kalman/set_logger_level"
	rosservice call --wait ${service_name} "logger: 'rosout' 
level: 'debug'"
fi

cat