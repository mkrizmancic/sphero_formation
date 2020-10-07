#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This module is used for utility and helper functions.

Classes:
    Vector2: 2D vector class representation with x and y components
    MarkerSet: convenience class for handling interactive Rviz markers

Function:
    pose_dist: calculate distance between two ROS Pose type variables
"""

import math
import rospy
import logging
import numpy as np
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Vector3, Quaternion
from tf.transformations import quaternion_from_euler


class Vector2(object):
    """
    2D vector class representation with x and y components.

    Supports simple addition, subtraction, multiplication, division and
    normalization, as well as getting norm and angle of the vector and
    setting limit and magnitude.

    Attributes:
        x (float): x component of the vector
        y (float): y component of the vector

    Methods:
        norm(self): Return the norm of the vector
        arg(self): Return the angle of the vector
        normalize(self): Normalize the vector
        limit(self, value): Limit vector's maximum magnitude to given value
        set_mag(self, value): Set vector's magnitude without changing direction
    """

    def __init__(self, x=0, y=0):
        """
        Initialize vector components.

        Args:
            x (float): x component of the vector
            y (float): y component of the vector
        """
        self.x = x
        self.y = y

    @classmethod
    def from_norm_arg(cls, norm=0, arg=0):
        inst = cls(1, 1)
        inst.set_mag(norm)
        inst.set_angle(arg)
        return inst

    def __add__(self, other):
        if isinstance(other, self.__class__):
            return Vector2(self.x + other.x, self.y + other.y)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x + other, self.y + other)

    def __sub__(self, other):
        if isinstance(other, self.__class__):
            return Vector2(self.x - other.x, self.y - other.y)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x - other, self.y - other)

    def __div__(self, other):
        if isinstance(other, self.__class__):
            raise ValueError("Cannot divide two vectors!")
        elif isinstance(other, int) or isinstance(other, float):
            if other != 0:
                return Vector2(self.x / other, self.y / other)
            else:
                return Vector2()

    def __mul__(self, other):
        if isinstance(other, self.__class__):
            raise NotImplementedError("Multiplying vectors is not implemented!")
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x * other, self.y * other)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __str__(self):
        return "({: .5f}, {: 6.1f})".format(self.norm(), self.arg())
        # return "({: .3f}, {: .3f})".format(self.x, self.y)

    def __repr__(self):
        return "Vector2({0}, {1})\n\t.norm = {2}\n\t.arg = {3}".format(self.x, self.y, self.norm(), self.arg())

    def norm(self):
        """Return the norm of the vector."""
        return math.sqrt(pow(self.x, 2) + pow(self.y, 2))

    def arg(self):
        """Return the angle of the vector."""
        return math.degrees(math.atan2(self.y, self.x))

    def set_mag(self, value):
        """Set vector's magnitude without changing direction."""
        if self.norm() == 0:
            logging.warning('Trying to set magnitude for a null-vector! Angle will be set to 0!')
            self.x = 1
            self.y = 0
        else:
            self.normalize()
        self.x *= value
        self.y *= value

    def set_angle(self, value):
        """Set vector's direction without changing magnitude."""
        if self.norm() == 0:
            logging.warning('Trying to set angle for a null-vector! Magnitude will be set to 1!')
            self.x = 1
            self.y = 0
        delta = angle_diff(self.arg(), value)
        self.rotate(delta)

    def rotate(self, value):
        """Rotate vector by degrees specified in value."""
        value = math.radians(value)
        self.x, self.y = math.cos(value) * self.x - math.sin(value) * self.y, \
                         math.sin(value) * self.x + math.cos(value) * self.y

    def normalize(self, ret=False):
        """Normalize the vector."""
        d = self.norm()
        if d:
            if not ret:
                self.x /= d
                self.y /= d
            else:
                return Vector2(self.x / d, self.y / d)

    def limit(self, value):
        """Limit vector's maximum magnitude to given value."""
        if self.norm() > value:
            self.set_mag(value)

    def limit_lower(self, value):
        """Limit vector's minimum magnitude to given value."""
        if self.norm() < value:
            self.set_mag(value)

    def constrain(self, old_value, max_value):
        """Limit vector's change of direction to max_value from old_value."""
        desired_value = self.arg()
        delta = angle_diff(old_value, desired_value)
        if abs(delta) > max_value:
            value = angle_diff(desired_value, old_value + math.copysign(max_value, delta))
            self.rotate(value)


def angle_diff(from_angle, to_angle):
    diff = (to_angle - from_angle) % 360
    if diff >= 180:
        diff -= 360
    return diff


def pose_dist(pose1, pose2):
    """Return Euclidean distance between two ROS poses."""
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y

    return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))


class MarkerSet(object):
    """
    Convenience class for handling Rviz markers.

    Markers are used to visualize each of the Reynolds' rules component in Rviz.
    Markers are set to arrows to represent force and velocity vectors.
    """
    def __init__(self):
        """Initialize class and set common marker properties."""
        self.visualization = MarkerArray()

        # Make sure these keys are the same as the ones in `boids.py`
        keys = ['alignment', 'cohesion', 'separation', 'avoid', 'acceleration', 'velocity', 'estimated']
        self.markers = dict.fromkeys(keys)

        marker_id = 0
        for key in keys:
            self.markers[key] = Marker()
            self.markers[key].header.frame_id = rospy.get_namespace() + 'base_link'
            self.markers[key].header.stamp = rospy.get_rostime()
            self.markers[key].ns = rospy.get_namespace().split('/')[1]
            self.markers[key].id = marker_id
            self.markers[key].type = Marker.ARROW
            self.markers[key].action = Marker.ADD
            self.markers[key].pose = Pose()
            self.markers[key].pose.position.z = 0.036  # Sphero radius
            self.markers[key].lifetime = rospy.Duration(0)
            self.markers[key].frame_locked = True
            marker_id += 1

        # Set colors of each marker
        self.markers['alignment'].color = ColorRGBA(0, 0, 1, 1)     # blue
        self.markers['cohesion'].color = ColorRGBA(0, 1, 0, 1)      # green
        self.markers['separation'].color = ColorRGBA(1, 0, 0, 1)    # red
        self.markers['avoid'].color = ColorRGBA(1, 1, 0, 1)         # yellow
        self.markers['acceleration'].color = ColorRGBA(0, 0, 0, 1)  # black
        self.markers['velocity'].color = ColorRGBA(1, 1, 1, 1)      # white
        self.markers['estimated'].color = ColorRGBA(1, 0.55, 0, 1)  # orange

    def update_data(self, values):
        """
        Set scale and direction of markers.

        Args:
            values (dict): Holds norm and arg data for each component
        """
        if values is not None:
            for key in self.markers.keys():
                data = values[key]
                angle = Quaternion(*quaternion_from_euler(0, 0, math.radians(data.arg())))
                scale = Vector3(data.norm(), 0.02, 0.02)

                self.markers[key].header.stamp = rospy.get_rostime()
                self.markers[key].pose.orientation = angle
                self.markers[key].scale = scale

            self.visualization.markers = self.markers.values()
        return self.visualization


class MAFilter(object):
    # TODO: remove if not necessary
    """Implementation of a moving average filter with variable window length."""
    def __init__(self, win_length):
        """
        Initialize empty window for averaging.

        Args:
            win_length: length of a window
        """
        # Window is initialized with NaNs so that the average would be correct
        # during the first few steps while the window is not yet full
        self.window = np.array([np.nan] * win_length)

    def step(self, value):
        """
        Add new value at the end of the window, shift older values to the left
        and return the average.
        """
        self.window[:-1] = self.window[1:]
        self.window[-1] = value
        # np.nanmean returns mean value while ignoring NaNs
        return np.nanmean(self.window)
