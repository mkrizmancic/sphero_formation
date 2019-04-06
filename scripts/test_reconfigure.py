#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from dynamic_reconfigure.server import Server
from sphero_formation.cfg import TestConfig


class TestReconf():
    """
    Dynamic reconfigure server.

    Process parameter changes from rqt_reconfigure and update parameter server.
    Publish empty message to let other nodes know there are updated parameters
    on server.
    """
    def __init__(self):
        """Initialize dynamic reconfigure server."""
        Server(TestConfig, self.callback)

        # Keep program from exiting
        rospy.spin()

    def callback(self, config, level):
        """Display all parameters when changed and signal to update."""
        rospy.loginfo("[Test reconfigure] => \n" +
                      """\tReconfigure Request:
                        Test duration: {test_duration}
                        Setpoint speed: {setpoint_speed}
                        Heading change: {heading_change}
                        Heading step time: {heading_time}
                        Max turning rate: {turning_rate}""".format(**config))
        return config


if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("test_reconf", anonymous=False)

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        dr = TestReconf()
    except rospy.ROSInterruptException:
        pass
