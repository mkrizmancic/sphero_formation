#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from geometry_msgs.msg import Twist

from util import Vector2

class SpheroTest(object):
    def __init__(self):
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.frequency = 10

    def start(self):
        duration = rospy.get_param('/test_reconf/test_duration') * self.frequency
        hdg_change_step = rospy.get_param('/test_reconf/heading_time') * self.frequency
        stp_speed = rospy.get_param('/test_reconf/setpoint_speed')
        hdg_change = rospy.get_param('/test_reconf/heading_change')
        turning_rate = rospy.get_param('/test_reconf/turning_rate')
        current_hdg = 0

        cmd_vel = Twist()
        velocity = Vector2.from_norm_arg(stp_speed, 0)

        step = 0
        rate = rospy.Rate(self.frequency)
        while step < duration:
            if step >= hdg_change_step:
                if hdg_change - current_hdg > turning_rate:
                    current_hdg += turning_rate
                    velocity = Vector2.from_norm_arg(stp_speed, current_hdg)
                else:
                    velocity = Vector2.from_norm_arg(stp_speed, hdg_change)

            cmd_vel.linear.x = velocity.x
            cmd_vel.linear.y = velocity.y
            self.pub_vel.publish(cmd_vel)
            step += 1
            rate.sleep()
