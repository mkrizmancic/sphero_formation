#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import ColorRGBA
from sphero_formation.srv import *


class InitializationNode():

    def callback(self, data):
        self.current_position = data.poses[0]

    def handle_init(self, req):
        return_value = self.initial_positions[req.key]
        return ReturnInitialsResponse(return_value)

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""

        # Get the number of agents
        self.num_robots = rospy.get_param("/sphero_init/num_of_robots", 4)

        # Crete publishers for sending color commands
        pub_keys = ['/sphero_{}/'.format(i) for i in range(self.num_robots)]
        self.pubs = dict.fromkeys(pub_keys)
        for key in self.pubs.keys():
            self.pubs[key] = rospy.Publisher(key + 'set_color_', ColorRGBA, queue_size=1)

        # Initialize class variables
        self.initial_positions = dict.fromkeys(pub_keys, Pose())
        self.current_position = Pose()

        # Create a subscriber
        rospy.Subscriber('/mocap_node/locations', PoseArray, self.callback, queue_size=1)

        # Main while loop.
        for key in self.pubs.keys():  # For each Sphero..
            print 'Light up ', key
            rospy.sleep(2)
            self.pubs[key].publish(1.0, 1.0, 1.0, 1.0)  # ..turn on LEDs..
            rospy.sleep(2)
            self.initial_positions[key] = self.current_position  # ..get its position..
            print self.current_position.position, '\n'
            self.pubs[key].publish(0.0, 0.0, 0.0, 1.0)  # ..turn off LEDs..

        # Create a service server
        rospy.Service('return_initials', ReturnInitials, self.handle_init)

        rospy.spin()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('initialization')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        i = InitializationNode()
    except rospy.ROSInterruptException:
        pass
