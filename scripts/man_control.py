#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, ColorRGBA


class ManControlNode():
    def joystick_callback(self, data):
        """Receive inputs from joystick and convert them to control signals."""

        if data.buttons[5]:
            if self.driving_mode == 'analog':
                self.driving_mode = 'digital'
                print "Driving mode: digital"
            else:
                self.driving_mode = 'analog'
                print "Driving mode: analog"

        if self.driving_mode == 'analog':
            # Sphero driver prima vrijednosti brzine u rasponu 0-255
            self.cmd_vel.linear.y = int(data.axes[1] * 255 * self.sensitivity)
            self.cmd_vel.linear.x = -int(data.axes[0] * 255 * self.sensitivity)
            print "\n", self.cmd_vel.linear

        elif self.driving_mode == 'digital':
            self.cmd_vel.linear.y = int(data.axes[5] * self.real_vel_stp * 100)
            self.cmd_vel.linear.x = -int(data.axes[4] * self.real_vel_stp * 100)
            print "\n", self.cmd_vel.linear

        # RGB LED colors
        if data.buttons[1]:  # green: green button A
            self.pub_rgb_led.publish(0.0, 255.0, 0.0, 1.0)
        elif data.buttons[2]:  # red: red button B
            self.pub_rgb_led.publish(255.0, 0.0, 0.0, 1.0)
        elif data.buttons[3]:  # yellow: yellow button Y
            self.pub_rgb_led.publish(255.0, 255.0, 0.0, 1.0)
        elif data.buttons[0]:  # blue: blue button X
            self.pub_rgb_led.publish(0.0, 0.0, 255.0, 1.0)
        elif data.buttons[9]:  # white: start button
            self.pub_rgb_led.publish(255.0, 255.0, 255.0, 1.0)
        elif data.buttons[8]:  # black: end button
            self.pub_rgb_led.publish(0.0, 0.0, 0.0, 1.0)

        if data.buttons[4]:
            self.real_vel_stp += 0.1
            if self.real_vel_stp > 3:
                self.real_vel_stp = 3
            print "Real speed setpoint: ", self.real_vel_stp
        elif data.buttons[6]:
            self.real_vel_stp -= 0.1
            if self.real_vel_stp < 0:
                self.real_vel_stp = 0
            print "Real speed setpoint: ", self.real_vel_stp

        # # Hold L1 and select heading using right stick
        # if data.buttons[4] == 1:
        #     # Calculate heading, convert to degrees and wrap to [0, 359]
        #     self.heading = (math.degrees(math.atan2(data.axes[2], data.axes[3])) + 180) % 360
        #     self.pub_hdg.publish(self.heading)

        # if data.buttons[5]:
        #     self.pub_stab.publish(1)  # disable stabilization
        # elif data.buttons[7]:
        #     self.pub_stab.publish(0)  # enable stabilization

    def __init__(self):
        # Create a publisher for commands
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # self.pub_hdg = rospy.Publisher('set_heading', Float32, queue_size=1)
        # self.pub_b_led = rospy.Publisher('set_back_led', Float32, queue_size=1)
        self.pub_rgb_led = rospy.Publisher('set_color', ColorRGBA, queue_size=1)
        # self.pub_stab = rospy.Publisher('disable_stabilization', Bool, queue_size=1)

        # Set class variables
        self.sensitivity = rospy.get_param('~sensitivity', 0.2)
        self.real_vel_stp = 0.5
        self.driving_mode = 'analog'

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
