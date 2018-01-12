#!/usr/bin/env python

import rospy

from dynamic_reconfigure.server import Server
from sphero_formation.cfg import ReynoldsConfig


def callback(config, level):
    rospy.loginfo("""Reconfigure Request:
        Alignment: {alignment_factor}
        Cohesion: {cohesion_factor}
        Separation: {separation_factor}
        Max speed: {max_speed}
        Crowd radius: {crowd_radius}
        Search radius: {search_radius}
        Avoid radius: {avoid_radius}""".format(**config))
    return config


if __name__ == "__main__":
    rospy.init_node("dyn_reconf", anonymous=False)

    srv = Server(ReynoldsConfig, callback)
    rospy.spin()
