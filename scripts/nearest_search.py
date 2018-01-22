#!/usr/bin/env python

import rospy
import math
from copy import deepcopy
import message_filters as mf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sphero_formation.msg import OdometryArray


def get_distance(a, b):
    return math.sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2))


class NearestSearch():

    def callback(self, *data):
        search_radius = rospy.get_param('/dyn_reconf/search_radius')
        sending = {}

        # OVO MOZDA RADI, A MOZDA I NE, TREBA PROVJERITI, ALI UGL.
        # TREBALO BI ZA SVAKOG ROBOTA TRAZITI KOJI SU MU NAJBLIZI I SLATI
        # POZICIJE TIH ROBOTA U KOORDINATNOM SUSTAVU ONOG ZA KOJI SE TRAZE
        # NAJBLIZI
        for sample in data:
            key = sample.header.frame_id.split('/')[1]
            agent_position = sample.pose.pose.position
            sending[key] = OdometryArray()
            for sample2 in data:
                friend_position = sample2.pose.pose.position
                distance = get_distance(agent_position, friend_position)
                if distance > 0 and distance <= search_radius:
                    rel_pos = deepcopy(sample2)
                    rel_pos.pose.pose.position.x = friend_position.x - agent_position.x
                    rel_pos.pose.pose.position.y = friend_position.y - agent_position.y
                    sending[key].array.append(rel_pos)

        for key in self.pubs.keys():
            self.pubs[key].publish(sending[key])

    def __init__(self):
        self.num_agents = 4

        # Create a publisher for commands
        pub_keys = ['robot_{}'.format(i) for i in range(self.num_agents)]
        self.pubs = dict.fromkeys(pub_keys)
        for key in self.pubs.keys():
            self.pubs[key] = rospy.Publisher('/' + key + '/nearest', OdometryArray, queue_size=1)

        # Set the message to publish as command.
        self.cmd_vel = Twist()
        self.dummy = OdometryArray()

        # Create a subscriber
        subs = [mf.Subscriber("/robot_{}/odom".format(i), Odometry) for i in range(self.num_agents)]
        self.ts = mf.TimeSynchronizer(subs, 10)
        self.ts.registerCallback(self.callback)

        # Main while loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Publish our command.
            rate.sleep()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('NearestSearch')

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        ns = NearestSearch()
    except rospy.ROSInterruptException:
        pass
