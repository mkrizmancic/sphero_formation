#!/usr/bin/env python

import rospy
from dynamic_reconfigure.server import Server
from sphero_formation.cfg import ReynoldsConfig


def callback(config, level):
    """Display all parameters when changed."""
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
    return config


if __name__ == "__main__":
    # Initialize the node and name it.
    rospy.init_node("dyn_reconf", anonymous=False)

    # Start dynamic reconfigure server
    srv = Server(ReynoldsConfig, callback)
    rospy.spin()
