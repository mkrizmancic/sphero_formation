#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, ColorRGBA, Bool
import math


class ManControlNode():
    def joystick_callback(self, data):
        """Receive inputs from joystick and convert them to control signals."""

        # Sphero driver prima vrijednosti brzine u rasponu 0-255
        self.cmd_vel.linear.y = int(data.axes[1] * 255 * self.sensitivity)
        self.cmd_vel.linear.x = -int(data.axes[0] * 255 * self.sensitivity)
        #print (rospy.get_name() + "  cmd_vel: ", self.cmd_vel)

        # Back LED: blue button X
        if data.buttons[0]:
            self.pub_b_led.publish(1.0)  # 0.0 - 1.0
        else:
            self.pub_b_led.publish(0.0)  # 0.0 - 1.0

        # RGB LED colors
        if data.buttons[1]:  # green: green button A
            self.pub_rgb_led.publish(0.0, 255.0, 0.0, 1.0)
        elif data.buttons[2]:  # red: red button B
            self.pub_rgb_led.publish(255.0, 0.0, 0.0, 1.0)
        elif data.buttons[3]:  # yellow: yellow button Y
            self.pub_rgb_led.publish(255.0, 255.0, 0.0, 1.0)

        # Hold L1 and select heading using right stick
        if data.buttons[4] == 1:
            # Calculate heading, convert to degrees and wrap to [0, 359]
            self.heading = (math.degrees(math.atan2(data.axes[2], data.axes[3])) + 180) % 360
            self.pub_hdg.publish(self.heading)

        if data.buttons[5] == 1:
            self.pub_stab.publish(1) # disable stabilization
        elif data.buttons[7] == 1:
            self.pub_stab.publish(0) # enable stabilization

    def __init__(self):
        # Create a publisher for commands
        self.pub_vel = rospy.Publisher('cmd_vel_', Twist, queue_size=1)
        self.pub_hdg = rospy.Publisher('set_heading_', Float32, queue_size=1)
        self.pub_b_led = rospy.Publisher('set_back_led_', Float32, queue_size=1)
        self.pub_rgb_led = rospy.Publisher('set_color_', ColorRGBA, queue_size=1)
        self.pub_stab = rospy.Publisher('disable_stabilization_', Bool, queue_size=1)

        # Set class variables
        self.sensitivity = rospy.get_param('~sensitivity', 0.2)

        # Create a subscriber
        rospy.Subscriber("/joystick_input", Joy, self.joystick_callback, queue_size=1)

        # Initialize messages
        self.cmd_vel = Twist()
        self.heading = Float32()

        # Main while loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            self.pub_vel.publish(self.cmd_vel)
            rate.sleep()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Controller')

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        mcn = ManControlNode()
    except rospy.ROSInterruptException:
        pass
