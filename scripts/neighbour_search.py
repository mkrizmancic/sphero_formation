#!/usr/bin/env python

# Must import rospy and msgs
import rospy
import math
from copy import deepcopy
import message_filters
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from flocking.msg import OdometryArray


def get_distance(a, b):
    return math.sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2))


class NeighbourSearch():

    def callback(self, *data):
        search_radius = rospy.get_param('/dyn_reconf/search_radius')
        sending = {}

        #print data

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
                #print("{} - agent: {}\nfriend: {}\ndist: {}".format(key, agent_position, friend_position, distance))
                if distance > 0 and distance <= search_radius:
                    rel_pos = deepcopy(sample2)
                    rel_pos.pose.pose.position.x = friend_position.x - agent_position.x
                    rel_pos.pose.pose.position.y = friend_position.y - agent_position.y
                    sending[key].array.append(rel_pos)

        for key in self.pubs.keys():
            self.pubs[key].publish(sending[key])

    # Must have __init__(self) function for a class
    def __init__(self):

        # Create a publisher for commands

        # Set the message to publish as command.
        # self.variable means you can access it from class fnc
        self.cmd_vel = Twist()
        self.dummy = OdometryArray()

        # Create a subscriber for color msg
        self.number_of_agents = 4
        subs = []
        for i in range(self.number_of_agents):
            subs.append(message_filters.Subscriber("/robot_{}/odom".format(i), Odometry))

        pub_keys = ['robot_{}'.format(i) for i in range(self.number_of_agents)]
        self.pubs = dict.fromkeys(pub_keys)
        for key in self.pubs.keys():
            self.pubs[key] = rospy.Publisher('/' + key + '/neighbours', OdometryArray, queue_size=1)

        self.ts = message_filters.TimeSynchronizer(subs, 10)
        self.ts.registerCallback(self.callback)

        # Main while loop.
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # Publish our command.
            rate.sleep()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('NeighbourSearch')

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        rc = NeighbourSearch()
    except rospy.ROSInterruptException:
        pass
