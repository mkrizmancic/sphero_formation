#!/usr/bin/env python
"""TODO: MISSING DOCSTRING!"""

import rospy
import message_filters as mf
from geometry_msgs.msg import Twist, PoseArray
from boids import Boid
from sphero_formation.msg import OdometryArray


class ReynoldsController():
    """TODO: MISSING DOCSTRING!"""

    def callback(self, *data):
        """TODO: MISSING DOCSTRING!"""
        my_agent = data[0].array[0]
        nearest_agents = data[0].array[1:]
        obstacles = data[1].poses

        # Compute agent's velocity and publish the command
        cmd_vel = self.agent.compute_velocity(my_agent, nearest_agents, obstacles)
        self.pub.publish(cmd_vel)

    def __init__(self):
        """TODO: MISSING DOCSTRING!"""

        # Create a publisher for commands
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Create a subscriber
        subs = [mf.Subscriber("nearest", OdometryArray), mf.Subscriber("avoid", PoseArray)]
        self.ts = mf.TimeSynchronizer(subs, 10)
        self.ts.registerCallback(self.callback)

        # Initialize other variables
        self.agent = Boid()

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
