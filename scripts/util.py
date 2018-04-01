#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
This module is used for utility and helper functions.

Classes:
    Vector2: 2D vector class representation with x and y components
"""

import math


class Vector2():
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
