#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import geometry_msgs.msg
from nav_msgs.msg import Odometry


def callback(msg):
    global tfBuffer
    try:
        tf = geometry_msgs.msg.TransformStamped()
        tf.child_frame_id = msg.header.frame_id
        tf.header.frame_id = "map"
        tf.header.stamp = msg.header.stamp

        tf.transform.translation.x = 0
        tf.transform.translation.y = 0
        tf.transform.translation.z = 0

        tf.transform.rotation.x = 0
        tf.transform.rotation.y = 0
        tf.transform.rotation.z = 0
        tf.transform.rotation.w = 1

        broadcaster.sendTransform(tf)

    except BaseException as exc:
        print exc
        tfBuffer.clear()
        return


if __name__ == '__main__':
    rospy.init_node('simulation_tf')
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    broadcaster = tf2_ros.TransformBroadcaster()

    num_of_robots = rospy.get_param("~num_of_robots")
    [rospy.Subscriber("/robot_{}/odom".format(i), Odometry, callback) for i in range(num_of_robots)]

    rospy.spin()
