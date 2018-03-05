#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, PoseArray
from sphero_formation.msg import OdometryArray
import message_filters as mf
from boids import Boid


class ReynoldsController():

    def callback(self, *data):
        self.cmd_vel = self.agent.compute_velocity(data[0].array, data[1].poses)

    def __init__(self):

        # Create a publisher for commands
        pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        # Set the message to publish as command
        self.cmd_vel = Twist()

        # Create a subscriber
        # rospy.Subscriber("nearest", OdometryArray, self.callback, queue_size=1)
        subs = [mf.Subscriber("nearest", OdometryArray), mf.Subscriber("avoid", PoseArray)]
        self.ts = mf.TimeSynchronizer(subs, 10)
        self.ts.registerCallback(self.callback)

        # Initialize other variables
        self.agent = Boid()

        # Main while loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Publish our command
            pub.publish(self.cmd_vel)
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
