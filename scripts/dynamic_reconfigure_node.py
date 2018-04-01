#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from dynamic_reconfigure.server import Server
from sphero_formation.cfg import ReynoldsConfig
from std_msgs.msg import Empty


class DynReconf():
    """
    Dynamic reconfigure server.

    Process parameter changes from rqt_reconfigure and update parameter server.
    Publish empty message to let other nodes know there are updated parameters
    on server.
    """
    def __init__(self):
        """Initialize publisher and dynamic reconfigure server."""
        # Create publisher
        self.pub = rospy.Publisher('param_update', Empty, queue_size=1)

        # Start dynamic reconfigure server
        Server(ReynoldsConfig, self.callback)

        # Main while loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

    def callback(self, config, level):
        """Display all parameters when changed and signal to update."""
        rospy.loginfo(rospy.get_caller_id() +
                      """\nReconfigure Request:
                        Alignment: {alignment_factor}
                        Cohesion: {cohesion_factor}
                        Separation: {separation_factor}
                        Avoid: {avoid_factor}
                        Max speed: {max_speed}
                        Max force: {max_force}
                        Friction: {friction}
                        Crowd radius: {crowd_radius}
                        Search radius: {search_radius}
                        Avoid radius: {avoid_radius}""".format(**config))
        self.pub.publish()
        return config


if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("dyn_reconf", anonymous=False)

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        dr = DynReconf()
    except rospy.ROSInterruptException:
        pass
