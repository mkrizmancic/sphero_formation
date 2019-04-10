#!/usr/bin/env python
# -*- coding: utf-8 -*-

import tf
import rospy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry

from kalman_filter import KalmanFilter
from util import pose_dist
from sphero_formation.srv import *


class KalmanFilterNode(object):
    """
    ROS node implementation of Kalman filter.

    This node subscribes to a list of all existing Sphero's positions
    broadcast from OptiTrack system, associates one of them to the Sphero in
    the same namespace and uses Kalman filter to output steady position and
    velocity data for other nodes.
    """
    def get_initial_position(self):
        """Calls service which returns Sphero's initial position."""
        rospy.wait_for_service('/return_initials')
        try:
            get_initials = rospy.ServiceProxy('/return_initials', ReturnInitials)
            response = get_initials(rospy.get_namespace())
            return response.initial
        except rospy.ServiceException, e:
            rospy.logerr(rospy.get_name() + ": Service call failed: %s", e)

    def sensor_callback(self, data):
        """Process received positions data and return Kalman estimation.
        :type data: Odometry
        """

        if self.initial_run:
            initial_pos = data.pose.pose
            # Initialize Kalman filter and estimation
            self.filter = KalmanFilter(1.0 / self.sub_frequency, initial_pos)
            self.X_est = Odometry()
            self.X_est.pose.pose = initial_pos
            self.initial_run = False
        else:
            X_measured = data.pose.pose
            vX_measured = data.twist.twist

            # If measurement data is not available, use only prediction step
            # Else, use prediction and update step
            if X_measured is None:
                self.X_est = self.filter.predict(u=None)
            else:
                self.filter.predict(u=None)
                self.X_est = self.filter.update(X_measured)

            X_estimated = self.X_est.pose.pose
            vX_estimated = self.X_est.twist.twist

            rospy.logdebug(' x: % 7.5f, % 7.5f, %+7.5f', X_measured.position.x, X_estimated.position.x, X_measured.position.x - X_estimated.position.x)
            rospy.logdebug(' y: % 7.5f, % 7.5f, %+7.5f', X_measured.position.y, X_estimated.position.y, X_measured.position.y - X_estimated.position.y)
            rospy.logdebug('vx: % 7.5f, % 7.5f, %+7.5f', vX_measured.linear.x, vX_estimated.linear.x, vX_measured.linear.x - vX_estimated.linear.x)
            rospy.logdebug('vy: % 7.5f, % 7.5f, %+7.5f\n', vX_measured.linear.y, vX_estimated.linear.y, vX_measured.linear.y - vX_estimated.linear.y)

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""

        # Initialize class variables
        self.sub_frequency = 10

        # Create subscribers
        self.initial_run = True
        rospy.Subscriber('odom', Odometry, self.sensor_callback, queue_size=1)

        # Keep program from exiting
        rospy.spin()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Kalman')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        kf = KalmanFilterNode()
    except rospy.ROSInterruptException:
        pass
