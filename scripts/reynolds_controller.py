#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import pandas as pd
import message_filters as mf
from dynamic_reconfigure.msg import Config
from geometry_msgs.msg import Twist, PoseArray
from visualization_msgs.msg import MarkerArray

from boids import Boid
from util import MarkerSet
from sphero_formation.msg import OdometryArray


class ReynoldsController(object):
    """
    ROS node implementation of Reynolds' flocking algorithm.

    This node represents a single agent in the flock. It subscribes to the list
    of other agents within search radius. Velocity of the agent is calculated
    based on Reynolds' flocking rules and this information is published to the
    simulator or physical implementation of the agent.
    """

    def callback(self, *data):
        """
        Unpack received data, compute velocity and publish the result.

        This is a callback of message_filters TimeSynchronizer subscriber. It is
        called only when all defined messages arrive with the same time stamp.
        In this case, there are two messages: "nearest" of type OdometryArray
        and "avoid" of type PoseArray. `data` is a list containing data from
        these messages.
            `data[0]` contains neighboring agents
            `data[1]` contains positions of obstacles
        """
        my_agent = data[0].array[0]         # odometry data for this agent is first in list
        nearest_agents = data[0].array[1:]  # odometry data for neighbors follows
        obstacles = data[1].poses           # store obstacles

        if self.params_set:
            # Compute agent's velocity and publish the command.
            ret_vel, viz = self.agent.compute_velocity(my_agent, nearest_agents, obstacles)

            # This is for use with real robots (Spheros).
            if self.run_type == 'real':
                cmd_vel = Twist()
                cmd_vel.linear.x = int(ret_vel.linear.x * 100)
                cmd_vel.linear.y = int(ret_vel.linear.y * 100)
                self.cmd_vel_pub.publish(cmd_vel)
            # This is for use with simulation.
            elif self.run_type == 'sim':
                self.cmd_vel_pub.publish(ret_vel)

            # Publish markers for visualization in Rviz.
            marker_array = self.markers.update_data(viz)
            self.markers_pub.publish(marker_array)

    def param_callback(self, data):
        """Call method for updating flocking parameters from server."""
        param_names = ['alignment_factor', 'cohesion_factor', 'separation_factor', 'avoid_factor',
                       'max_speed', 'max_force', 'friction', 'crowd_radius',
                       'search_radius', 'avoid_radius']
        # Dictionary for passing parameters.
        param_dict = {param: rospy.get_param('/dyn_reconf/' + param) for param in param_names}
        self.agent.update_parameters(param_dict)
        self.params_set = True

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""
        # Initialize class variables.
        init_vel_x = rospy.get_param("~init_vel_x", 0)
        init_vel_y = rospy.get_param("~init_vel_y", 0)
        frequency = rospy.get_param("/ctrl_loop_freq")
        wait_count = int(rospy.get_param("/wait_time") * frequency)
        start_count = int(rospy.get_param("/start_time") * frequency)
        self.run_type = rospy.get_param("/run_type")
        self.agent = Boid(init_vel_x, init_vel_y, wait_count, start_count, frequency)
        self.markers = MarkerSet()
        self.params_set = False

        # Create a publisher for commands.
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=frequency)
        self.markers_pub = rospy.Publisher('markers', MarkerArray, queue_size=frequency)

        # Create subscribers.
        rospy.Subscriber('/dyn_reconf/parameter_updates', Config, self.param_callback, queue_size=1)

        subs = [mf.Subscriber("nearest", OdometryArray), mf.Subscriber("avoid", PoseArray)]
        self.ts = mf.TimeSynchronizer(subs, 10)
        self.ts.registerCallback(self.callback)

        # Keep program from exiting
        rospy.spin()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ReynoldsController')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        rc = ReynoldsController()
    except rospy.ROSInterruptException:
        pass
