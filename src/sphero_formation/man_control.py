#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import colorsys
from random import random, randint, seed
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, ColorRGBA, Bool


class ManControlNode(object):
    """
    ROS node for manually controlling Spheros.

    Node subscribes to joystick input data and publishes commands to Sphero
    driver. `joystick_callback function gets called each time any of the
    joystick buttons or axes change states.
    """

    def joystick_callback(self, data):
        """Receive inputs from joystick and convert them to control signals."""

        # Joystick R3 button - enable/disable manual control
        if data.buttons[11]:
            if self.enabled:
                self.enabled = False
                if self.display_true:
                    rospy.loginfo("Manual control disabled")
            else:
                self.enabled = True
                if self.display_true:
                    rospy.loginfo("Manual control enabled")

        # Joystick L3 button - switch between analog and digital driving mode
        if data.buttons[10]:
            if self.driving_mode == 'analog':
                self.driving_mode = 'digital'
                if self.display_true:
                    rospy.loginfo("Driving mode: digital")
            else:
                self.driving_mode = 'analog'
                if self.display_true:
                    rospy.loginfo("Driving mode: analog")

        # Joystick L1 button - increase digital mode speed set point
        if data.buttons[4]:
            self.real_vel_stp += 0.1
            if self.real_vel_stp > 3:
                self.real_vel_stp = 3
            if self.display_true:
                rospy.loginfo("Real speed set point: %.1f", self.real_vel_stp)
        # Joystick L2 button - decrease digital mode speed set point
        elif data.buttons[6]:
            self.real_vel_stp -= 0.1
            if self.real_vel_stp < 0:
                self.real_vel_stp = 0
            if self.display_true:
                rospy.loginfo("Real speed set point: %.1f", self.real_vel_stp)

        # For analog driving - use left stick
        if self.driving_mode == 'analog':
            # Sphero driver accepts speeds in range 0-255
            self.cmd_vel.linear.y = int(data.axes[1] * 255 * self.sensitivity)
            self.cmd_vel.linear.x = -int(data.axes[0] * 255 * self.sensitivity)

        # For digital driving - use hat switch
        elif self.driving_mode == 'digital':
            self.cmd_vel.linear.y = int(data.axes[5] * self.real_vel_stp * 100)
            self.cmd_vel.linear.x = -int(data.axes[4] * self.real_vel_stp * 100)

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
        elif data.buttons[5]:  # random color for each Sphero: button R1
            colors = colorsys.hsv_to_rgb(random(), 1.0, 1.0)
            self.pub_rgb_led.publish(colors[0], colors[1], colors[2], 1.0)

        # Joystick R2 button - go in manual calibration mode
        if data.buttons[7]:
            if self.manual_calibration is False:
                if self.display_true:
                    rospy.loginfo("Manual calibration mode ON")
                self.was_enabled = self.enabled
                self.enabled = False
                self.manual_calibration = True
                self.pub_calibrate.publish(1)
            else:
                if self.display_true:
                    rospy.loginfo("Manual calibration mode OFF")
                self.manual_calibration = False
                self.pub_calibrate.publish(0)
                self.enabled = self.was_enabled

    def __init__(self):
        """Create subscribers and publishers and initialize class variables."""

        # Create publishers for commands
        self.pub_vel = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.pub_rgb_led = rospy.Publisher('set_color', ColorRGBA, queue_size=1)
        self.pub_calibrate = rospy.Publisher('manual_calibration', Bool, queue_size=1)

        # Set class variables
        self.sensitivity = rospy.get_param('~sensitivity', 0.2)  # left stick sensitivity
        self.frequency = rospy.get_param('/ctrl_loop_freq')
        self.real_vel_stp = 0.5
        self.driving_mode = 'analog'
        self.manual_calibration = False
        self.was_enabled = self.enabled = False

        # Work-around for displaying info messages. Currently, each Sphero has
        # its own manual control node even though they all get same commands.
        # Because of that, multiple info messages would appear on screen,
        # one for each Sphero. This work-around ensures that only one message
        # appears.
        if rospy.get_namespace() == '/sphero_0/':
            self.display_true = True
        else:
            self.display_true = False

        seed(randint(1, 1000))

        # Create a subscriber
        rospy.Subscriber("/joystick_input", Joy, self.joystick_callback, queue_size=1)

        # Initialize messages
        self.cmd_vel = Twist()
        self.heading = Float32()

        # Main while loop.
        rate = rospy.Rate(self.frequency)
        while not rospy.is_shutdown():
            if self.enabled:
                self.pub_vel.publish(self.cmd_vel)
            rate.sleep()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('man_control')

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        mcn = ManControlNode()
    except rospy.ROSInterruptException:
        pass
