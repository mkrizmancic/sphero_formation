#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry

from kalman_filter import KalmanFilter


class KalmanFilterNode(object):
    """
    ROS node implementation of Kalman filter.

    This node subscribes to a list of all existing Sphero's positions
    broadcast from OptiTrack system, associates one of them to the Sphero in
    the same namespace and uses Kalman filter to output steady position and
    velocity data for other nodes.
    """

    def sensor_callback(self, data):
        """Process received positions data and return Kalman estimation.
        :type data: Odometry
        """

        if self.initial_run:
            # Initialize Kalman filter and estimation.
            initial_pos = data.pose.pose
            self.filter = KalmanFilter(1.0 / self.sub_frequency, initial_pos)
            self.X_est = Odometry()
            self.X_est.pose.pose = initial_pos
            self.initial_run = False
        else:
            X_measured = data.pose.pose

            # If measurement data is not available, use only prediction step.
            # Else, use prediction and update step.
            if X_measured is None:
                self.X_est = self.filter.predict()
            else:
                self.X_est = self.filter.predict_update(X_measured)

            if self.debug_enabled:
                self.debug_pub.publish(self.X_est)

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""
        # Initialize class variables.
        self.sub_frequency = rospy.get_param('/data_stream_freq')
        self.pub_frequency = rospy.get_param('/ctrl_loop_freq')
        self.debug_enabled = rospy.get_param('/debug_kalman')
        self.X_est = None

        # Create publishers.
        pub = rospy.Publisher('odom_est', Odometry, queue_size=self.pub_frequency)
        if self.debug_enabled:
            self.debug_pub = rospy.Publisher('debug_est', Odometry, queue_size=self.sub_frequency)

        # Create subscribers.
        self.initial_run = True
        rospy.Subscriber('odom', Odometry, self.sensor_callback, queue_size=self.sub_frequency)

        # Don't try to publish anything before variable is set.
        while self.X_est is None:
            rospy.sleep(0.1)

        # Main while loop.
        rate = rospy.Rate(self.pub_frequency)
        while not rospy.is_shutdown():
            pub.publish(self.X_est)
            rate.sleep()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Kalman')

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        kf = KalmanFilterNode()
    except rospy.ROSInterruptException:
        pass
