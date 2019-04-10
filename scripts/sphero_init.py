#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import rostopic
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import ColorRGBA
from sphero_formation.srv import *


class InitializationNode(object):
    """
    ROS node responsible for providing initial Spheros positions.

    OptiTrack system is used to broadcast position of each Sphero. However,
    positions are stored in an unsorted list as it is impossible to label
    OptiTrack markers with IDs. This node starts a procedure to identify each
    Sphero and its initial position. Kalman filter node is later used to
    maintain this "one-on-one" relation and to provide positions. In order to
    identify a Sphero and its initial position, only one position can be present
    in mentioned unsorted list. Identification process consists of turning
    Sphero's LEDs on and off one by one while knowing which Sphero is the
    current one and storing the position in a dictionary.
    """

    def callback(self, data):
        """Store position data in class variable."""
        # `data` contains a list of poses. Store only the first one
        # This assumes that only one Sphero is lit up so its position is the
        # first and only one in the list
        self.current_position = data.poses[0]

    def handle_init(self, req):
        """Service handler. Return initial position upon request."""
        return_value = self.initial_positions[req.key]
        return ReturnInitialsResponse(return_value)

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""

        # Get the number of agents
        self.num_robots = rospy.get_param("/num_of_robots", 4)

        # Create publishers for sending color commands
        pub_keys = ['/sphero_{}/'.format(i) for i in range(self.num_robots)]
        self.pubs = dict.fromkeys(pub_keys)
        for key in self.pubs.keys():
            self.pubs[key] = rospy.Publisher(key + 'set_color', ColorRGBA, queue_size=1)

        # Initialize class variables
        self.initial_positions = dict.fromkeys(pub_keys, Pose())
        self.current_position = None

        # Calculate mocap node's publishing rate
        tr = rostopic.ROSTopicHz(-1)
        rospy.Subscriber('/mocap_node_positions', rospy.AnyMsg, tr.callback_hz, callback_args='/mocap_node/positions')
        rospy.sleep(3)
        mocap_pub_rate = 100 # int(round(tr.get_hz('/mocap_node/positions')[0]))
        rospy.set_param('/mocap_pub_rate', mocap_pub_rate)
        rospy.loginfo('Param \'/mocap_pub_rate\' set to %d', mocap_pub_rate)

        # Create a subscriber
        rospy.Subscriber('/mocap_node/positions', PoseArray, self.callback, queue_size=1)

        # Start identification process
        for key in self.pubs.keys():  # For each Sphero..
            rospy.sleep(1)
            rospy.loginfo('Light up ' + key)
            self.pubs[key].publish(1.0, 1.0, 1.0, 1.0)  # ..turn on LEDs..
            rospy.sleep(2)
            retry_count = 0
            # If found, position of the Sphero is in self.current_position.
            # If not, try to get the position two more times.
            while self.current_position is None and retry_count < 3:
                rospy.logwarn("Initial position not found for " + key)
                rospy.logwarn("Trying again...")
                self.pubs[key].publish(0.0, 0.0, 0.0, 1.0)  # .. turn off LEDs..
                rospy.sleep(1)
                self.pubs[key].publish(1.0, 1.0, 1.0, 1.0)  # ..turn on LEDs..
                rospy.sleep(2)
                retry_count += 1

            if self.current_position is not None:
                self.initial_positions[key] = self.current_position  # ..store its position..
                self.current_position = None
            else:
                rospy.logerr("Initial position not found for " + key)

            self.pubs[key].publish(0.0, 0.0, 0.0, 1.0)  # ..turn off LEDs..

        for key in self.pubs.keys():  # For each Sphero..
            rospy.sleep(0.5)
            self.pubs[key].publish(1.0, 1.0, 1.0, 1.0)  # ..turn on LEDs

        # Create a service server
        rospy.Service('return_initials', ReturnInitials, self.handle_init)

        # Keep program from exiting
        rospy.spin()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('initialization')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        IN = InitializationNode()
    except rospy.ROSInterruptException:
        pass
