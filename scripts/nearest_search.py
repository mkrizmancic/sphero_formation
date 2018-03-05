#!/usr/bin/env python

from __future__ import print_function
import rospy
import math
from copy import deepcopy
import message_filters as mf
from geometry_msgs.msg import Twist, PoseArray, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from sphero_formation.msg import OdometryArray


def get_distance(a, b):
    return math.sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2))


class NearestSearch():

    def pos_to_index(self, x_in, y_in):
        x = (x_in + self.origin.x) / self.resolution
        y = (y_in - self.origin.y) / self.resolution
        return int(x), int(y)

    def index_to_pos(self, x_in, y_in):
        x = x_in * self.resolution - self.origin.x
        y = y_in * self.resolution + self.origin.y
        return x, y

    def callback(self, *data):
        search_radius = rospy.get_param('/dyn_reconf/search_radius')
        avoid_radius = rospy.get_param('/dyn_reconf/avoid_radius')
        r = int(avoid_radius / self.resolution)
        sending = {}
        avoid = {}

        # OVO MOZDA RADI, A MOZDA I NE, TREBA PROVJERITI, ALI UGL.
        # TREBALO BI ZA SVAKOG ROBOTA TRAZITI KOJI SU MU NAJBLIZI I SLATI
        # POZICIJE TIH ROBOTA U KOORDINATNOM SUSTAVU ONOG ZA KOJI SE TRAZE
        # NAJBLIZI
        for sample in data:
            time = rospy.Time.now()
            key = sample.header.frame_id.split('/')[1]
            agent_position = sample.pose.pose.position
            sending[key] = OdometryArray()
            sending[key].header.stamp = time
            for sample2 in data:
                friend_position = sample2.pose.pose.position
                distance = get_distance(agent_position, friend_position)
                if distance > 0 and distance <= search_radius:
                    rel_pos = deepcopy(sample2)
                    rel_pos.pose.pose.position.x = friend_position.x - agent_position.x
                    rel_pos.pose.pose.position.y = friend_position.y - agent_position.y
                    sending[key].array.append(rel_pos)

            avoid[key] = PoseArray()
            avoid[key].header.stamp = time
            x0, y0 = self.pos_to_index(agent_position.x, agent_position.y)
            # print (x0, y0)
            x_range = range(max(0, x0 - r / 2), min(self.height, x0 + r / 2))
            y_range = range(max(0, y0 - r / 2), min(self.width, y0 + r / 2))
            # print (min(x_range), max(x_range))
            # print (min(y_range), max(y_range))
            for i in x_range:
                for j in y_range:
                    # print (self.map[i * self.width + j], end=' ')
                    if self.map[i * self.width + j] == 100:
                        x, y = self.index_to_pos(i, j)
                        obst = Pose()
                        obst.position.x = x - agent_position.x
                        obst.position.y = y - agent_position.y
                        avoid[key].poses.append(obst)
            #     print()
            # print()

        for key in self.nearest.keys():
            self.nearest[key].publish(sending[key])

        for key in self.avoid.keys():
            self.avoid[key].publish(avoid[key])

    def map_callback(self, data):
        self.map = data.data
        self.width = data.info.width
        self.height = data.info.height
        self.resolution = data.info.resolution
        self.origin = data.info.origin.position

    def __init__(self):
        self.num_agents = 4

        # Create a publisher for commands
        pub_keys = ['robot_{}'.format(i) for i in range(self.num_agents)]
        self.nearest = dict.fromkeys(pub_keys)
        for key in self.nearest.keys():
            self.nearest[key] = rospy.Publisher('/' + key + '/nearest', OdometryArray, queue_size=1)

        self.avoid = dict.fromkeys(pub_keys)
        for key in self.avoid.keys():
            self.avoid[key] = rospy.Publisher('/' + key + '/avoid', PoseArray, queue_size=1)

        # Set the message to publish as command.
        self.cmd_vel = Twist()
        self.dummy = OdometryArray()

        # Create a subscriber
        subs = [mf.Subscriber("/robot_{}/odom".format(i), Odometry) for i in range(self.num_agents)]
        self.ts = mf.TimeSynchronizer(subs, 10)
        self.ts.registerCallback(self.callback)

        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)

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
