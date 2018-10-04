#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This module is used for utility and helper functions.

Classes:
    Vector2: 2D vector class representation with x and y components
    MarkerSet: convenience class for handling interactive Rviz markers
"""

import rospy
import math
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

    def __repr__(self):
        return "({: 6.1f}, {: .5f})".format(self.arg(), self.norm())
        # return "({: .3f}, {: .3f})".format(self.x, self.y)

    def norm(self):
        """Return the norm of the vector."""
        return math.sqrt(pow(self.x, 2) + pow(self.y, 2))

    def arg(self):
        """Return the angle of the vector."""
        return math.degrees(math.atan2(self.y, self.x))

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

    def set_mag(self, value):
        """Set vector's magnitude without changing direction."""
        self.normalize()
        self.x *= value
        self.y *= value


def pose_dist(pose1, pose2):
    x1 = pose1.position.x
    y1 = pose1.position.y
    x2 = pose2.position.x
    y2 = pose2.position.y

    return math.sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2))


class MarkerSet(object):
    """docstring for MarkerSet"""
    def __init__(self):
        self.visualization = MarkerArray()

        keys = ['alignment', 'cohesion', 'separation', 'avoid', 'velocity']
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
            self.markers[key].pose.position.z = 0.075
            self.markers[key].lifetime = rospy.Duration(0)
            self.markers[key].frame_locked = True
            marker_id += 1

        self.markers['alignment'].color = ColorRGBA(0, 0, 1, 1)   # blue
        self.markers['cohesion'].color = ColorRGBA(0, 1, 0, 1)    # green
        self.markers['separation'].color = ColorRGBA(1, 0, 0, 1)  # red
        self.markers['avoid'].color = ColorRGBA(1, 1, 0, 1)       # yellow
        self.markers['velocity'].color = ColorRGBA(1, 1, 1, 1)    # white

    def update_data(self, values):
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
