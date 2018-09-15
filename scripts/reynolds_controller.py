#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import message_filters as mf
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, PoseArray
from boids import Boid
from sphero_formation.msg import OdometryArray


class ReynoldsController():
    """
    ROS node implementation of Reynolds' flocking algorithm.

    This node represents a single agent in the flock. It subscribes to the list
    of other agents within search radius. Velocity of the agent is calculated
    based on Reynolds' flocking rules and this information is published to the
    simulator or physical implementation of the agent.
    """

    def callback(self, *data):
        """Unpack received data, compute velocity and publish the result."""
        my_agent = data[0].array[0]
        nearest_agents = data[0].array[1:]
        obstacles = data[1].poses

        # Compute agent's velocity and publish the command
        if self.params_set:
            ret_vel = self.agent.compute_velocity(my_agent, nearest_agents, obstacles)
            cmd_vel = Twist()
            cmd_vel.linear.x = int(ret_vel.linear.x * 100)
            cmd_vel.linear.y = int(ret_vel.linear.y * 100)
            self.pub.publish(cmd_vel)

    def param_callback(self, data):
        """Call method for updating flocking parameters from server."""

        param_names = ['alignment_factor', 'cohesion_factor', 'separation_factor', 'avoid_factor',
                       'max_speed', 'max_force', 'friction', 'crowd_radius', 'search_radius']
        # Dictionary for passing parameters
        param_dict = {param: rospy.get_param('/dyn_reconf/' + param) for param in param_names}
        self.agent.update_parameters(param_dict)
        self.params_set = True

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""
        # Create a publisher for commands
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Initialize class variables
        init_vel_x = rospy.get_param("~init_vel_x", 0)
        init_vel_y = rospy.get_param("~init_vel_y", 0)
        self.agent = Boid(init_vel_x, init_vel_y)
        self.params_set = False

        # Create subscribers
        rospy.Subscriber('/param_update', Empty, self.param_callback, queue_size=1)

        subs = [mf.Subscriber("nearest", OdometryArray), mf.Subscriber("avoid", PoseArray)]
        self.ts = mf.TimeSynchronizer(subs, 10)
        self.ts.registerCallback(self.callback)

        # Main while loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('ReynoldsController')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        rc = ReynoldsController()
    except rospy.ROSInterruptException:
        pass
