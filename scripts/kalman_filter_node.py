#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseArray
from nav_msgs.msg import Odometry
from kalman_filter import KalmanFilter
from util import pose_dist
from sphero_formation.srv import *
import tf


class KalmanFilterNode():
    def get_initial_position(self):
        rospy.wait_for_service('/return_initials')
        try:
            get_initials = rospy.ServiceProxy('/return_initials', ReturnInitials)
            response = get_initials(rospy.get_namespace())
            return response.initial
        except rospy.ServiceException, e:
            rospy.logerr(rospy.get_name() + ": Service call failed: %s", e)

    def associate(self, data):
        X_measured = None
        # found = False
        for pose in data.poses:
            if pose_dist(self.X_est.pose.pose, pose) < self.sphero_radius:
                X_measured = pose
                self.missing_counter = 0
                # if found:
                #     rospy.logwarn("Multiple markers found for " + rospy.get_name())
                # found = True

        if X_measured is None:
            self.missing_counter += 1
            if self.missing_counter % 10 == 0 and self.missing_counter <= 50:
                rospy.logwarn(rospy.get_name() +
                              "marker mising for %d consecutive iterations.", self.missing_counter)
            elif self.missing_counter == 60:
                rospy.logerr(rospy.get_name() + "lost tracking!!")

        return X_measured

    def input_callback(self, data):
        pass

    def sensor_callback(self, data):
        X_measured = self.associate(data)
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
        self.missing_counter = 0
        self.sphero_radius = 0.06
        initial_pos = self.get_initial_position()
        rospy.loginfo(rospy.get_namespace() + initial_pos.position + '\n')

        self.filter = KalmanFilter(initial_pos)
        self.X_est = Odometry()
        self.X_est.pose.pose = initial_pos

        # Create subscribers
        # rospy.Subscriber('cmd_vel', Twist, self.input_callback, queue_size=1)
        rospy.Subscriber('/mocap_node/locations', PoseArray, self.sensor_callback, queue_size=1)

        # Create tf broadcaster
        br = tf.TransformBroadcaster()

        # Main while loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pub.publish(self.X_est)
            pos = self.X_est.pose.pose.position
            br.sendTransform((pos.x, pos.y, pos.z),
                             (0, 0, 0, 1),
                             rospy.Time.now(),
                             rospy.get_namespace(),
                             'map')
            rate.sleep()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('KalmanFilter')

    # Go to class functions that do all the heavy lifting
    # Do error checking
    try:
        kf = KalmanFilterNode()
    except rospy.ROSInterruptException:
        pass
