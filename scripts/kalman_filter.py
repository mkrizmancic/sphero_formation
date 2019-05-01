#!/usr/bin/env python
# -*- coding: utf-8 -*-
from __future__ import division
import rospy
import numpy as np
import numpy.linalg as npl
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class KalmanFilter(object):
    """Class implementation of Kalman filter."""

    def __init__(self, step_time, position, velocity=Twist()):
        """Initialize class variables and set initial conditions."""
        T = step_time
        self.X0 = self.get_numpy_state(position, velocity)
        self.P0 = 1e-6 * np.eye(4)

        q = 9
        Q = np.diag([q, q, q, q])
        L = np.diag([T ** 2 / 2, T ** 2 / 2, T, T])
        self.Q = L.dot(Q).dot(L.T)
        self.R = 1e-6 * np.eye(2)

        self.A = np.array([[1, 0, T, 0], [0, 1, 0, T], [0, 0, 1, 0], [0, 0, 0, 1]])
        self.H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]])

        self.X_old = self.X0
        self.P_old = self.P0

    def get_numpy_state(self, position, velocity):
        """Convert from some type (here: ROS msg) to numpy array."""
        x = position.position.x
        y = position.position.y
        vx = velocity.linear.x
        vy = velocity.linear.y
        state = np.array([[x, y, vx, vy]])
        return state.T

    def get_used_state(self, np_state):
        """Convert from numpy array to type used elsewhere (here: ROS msg)."""
        time = rospy.Time.now()
        msg = Odometry()
        msg.header.stamp = time
        msg.header.frame_id = rospy.get_namespace()
        msg.pose.pose.position.x = np_state[0][0]
        msg.pose.pose.position.y = np_state[1][0]
        msg.twist.twist.linear.x = np_state[2][0]
        msg.twist.twist.linear.y = np_state[3][0]
        return msg

    def predict(self, ret=True):
        """Kalman's prediction phase."""
        X_hat_m = np.dot(self.A, self.X_old)
        P_m = np.dot(np.dot(self.A, self.P_old), self.A.T) + self.Q

        self.X_old = X_hat_m
        self.P_old = P_m

        if ret:
            return self.get_used_state(X_hat_m)

    def update(self, Xm):
        """
        Kalman's update phase.

        Args:
            Xm: measured state vector
        """
        eye = np.eye(4)  # Identity matrix
        P_ = self.P_old  # P minus
        X_hat_m = self.X_old  # X hat minus
        Xm = self.get_numpy_state(Xm, Twist())

        # K = P(-)*H^T * [H*P(-)*H^T + R]^-1
        # X(+) = X(-) + K*[y - H*X(-)]
        # P(+) = [I-K*H] * P(-) * [I-K*H]^T + K*R*K^T
        K = P_.dot(self.H.T).dot(npl.inv(self.H.dot(P_).dot(self.H.T) + self.R))
        X_hat_p = X_hat_m + K.dot(self.H.dot(Xm) - self.H.dot(X_hat_m))
        P_p = (eye - K.dot(self.H)).dot(P_).dot((eye - K.dot(self.H)).T) + K.dot(self.R).dot(K.T)

        self.X_old = X_hat_p
        self.P_old = P_p

        return self.get_used_state(X_hat_p)

    def predict_update(self, Xm):
        """Call predict and update."""
        self.predict(ret=False)
        return self.update(Xm)
