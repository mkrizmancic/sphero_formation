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

    def associate(self, data):
        """
        Associate Sphero with its position and return it.

        Positions of all Spheros are determined using OptiTrack system and sent
        via mocap_node. It is currently impossible to label OptiTrack markers
        with some ID. Positions of all markers arrive in an unsorted list. We
        must associate each of the positions in list with a Sphero. To do this,
        we are looking which of the positions in the list is less than a Sphero
        radius away from the last available Sphero's position estimation. We
        assume that Sphero couldn't have traveled more than this distance
        between two consecutive position updates. OptiTrack is set to broadcast
        positions 100 times per second.
        """
        X_measured = None
        for pose in data.poses:
            if pose_dist(self.X_est.pose.pose, pose) < self.sphero_radius:
                X_measured = pose
                self.missing_counter = 0

        if X_measured is None:
            self.missing_counter += 1
            if self.missing_counter % 10 == 0 and self.missing_counter <= 90:
                rospy.logwarn(rospy.get_name() +
                              ": Marker missing for %d consecutive iterations.",
                              self.missing_counter)
            elif self.missing_counter == 100:
                rospy.logerr(rospy.get_name() + ": Lost tracking!!")

        return X_measured

    def sensor_callback(self, data):
        """Process received positions data and return Kalman estimation."""

        # Get measurement
        X_measured = self.associate(data)

        # If measurement data is not available, use only prediction step
        # Else, use prediction and update step
        if X_measured is None:
            self.X_est = self.filter.predict(u=None)
        else:
            self.filter.predict(u=None)
            self.X_est = self.filter.update(X_measured)

    def __init__(self):
        """Initialize agent instance, create subscribers and publishers."""
        # Create a publisher for commands
        pub = rospy.Publisher('odom', Odometry, queue_size=1)

        # Initialize class variables
        self.missing_counter = 0   # Counts iterations with missing marker information
        self.sphero_radius = 0.05  # Sphero radius in meters
        self.pub_frequency = rospy.get_param('/ctrl_loop_freq')
        self.sub_frequency = rospy.get_param('/mocap_pub_rate')
        initial_pos = self.get_initial_position()  # Get initial position
        rospy.loginfo(rospy.get_namespace() + '\n%s\n', initial_pos.position)

        # Initialize Kalman filter and estimation
        self.filter = KalmanFilter(1.0 / self.sub_frequency, initial_pos)
        self.X_est = Odometry()
        self.X_est.pose.pose = initial_pos

        # Create subscribers
        rospy.Subscriber('/mocap_node/positions', PoseArray, self.sensor_callback, queue_size=1)

        # Create tf broadcaster
        br = tf.TransformBroadcaster()

        # Main while loop.
        rate = rospy.Rate(self.pub_frequency)
        while not rospy.is_shutdown():
            pub.publish(self.X_est)
            pos = self.X_est.pose.pose.position
            br.sendTransform((pos.x, pos.y, pos.z),
                             (0, 0, 0, 1),
                             rospy.Time.now(),
                             rospy.get_namespace() + 'base_link',
                             'map')
            rate.sleep()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('Kalman')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        kf = KalmanFilterNode()
    except rospy.ROSInterruptException:
        pass
