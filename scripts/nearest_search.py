#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import math
import rospy
import message_filters as mf
from copy import deepcopy

from dynamic_reconfigure.msg import Config
from geometry_msgs.msg import PoseArray, Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from sphero_formation.msg import OdometryArray


def get_distance(a, b):
    """Return Euclidean distance between points a and b."""
    return math.sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2))


class NearestSearch(object):
    """
    Node that provides information about nearest flockmates and obstacles.

    Generally, Reynolds' flocking algorithm works on distributed systems.
    If agents don't have any sensors, a centralized system is needed. This node
    is an 'all-knowing' hub that makes virtual distributed system possible.
    It is subscribed to messages with position and velocity of each agent and
    knows the map layout. For each agent, it finds its neighbors within search
    radius and calculates their relative position. This data is then published
    to individual agents along side the list of obstacles within range.
    """

    def map_callback(self, data):
        """Save map occupancy grid and meta-data in class variables."""
        self.map = []
        self.map_width = data.info.width
        self.map_height = data.info.height
        self.map_resolution = data.info.resolution
        self.map_origin = data.info.origin.position

        # Reverse the order of rows in map
        for i in range(self.map_height - 1, -1, -1):
            self.map.append(data.data[i * self.map_width:(i + 1) * self.map_width])

    def pos_to_index(self, x_real, y_real):
        """Return list (map) indices for given real position coordinates."""
        col = (x_real - self.map_origin.x) / self.map_resolution
        row = (self.map_origin.y + self.map_height *
               self.map_resolution - y_real) / self.map_resolution
        return int(col), int(row)

    def index_to_pos(self, row, col):
        """Return real position coordinates for list (map) indices."""
        x_real = self.map_origin.x + col * self.map_resolution
        y_real = self.map_origin.y + (self.map_height - row) * self.map_resolution
        return x_real, y_real

    def param_callback(self, data):
        """Update search parameters from server."""
        while not rospy.has_param('/dyn_reconf/search_radius'):
            rospy.sleep(0.1)

        self.search_radius = rospy.get_param('/dyn_reconf/search_radius')
        avoid_radius = rospy.get_param('/dyn_reconf/avoid_radius')
        self.r = int(avoid_radius / self.map_resolution)

    def robot_callback(self, *data):
        """Find and publish positions of nearest agents and obstacles."""

        for agent in data:
            # Get current time and observed agent's name
            time = rospy.Time.now()
            key = agent.header.frame_id.split('/')[1]

            # Get agent's position and initialize relative positions message
            agent_position = agent.pose.pose.position
            nearest_agents = OdometryArray()
            nearest_agents.header.stamp = time

            # Append odometry data from observed agent to the publishing message
            nearest_agents.array.append(deepcopy(agent))

            # For all other agents calculate the distance from the observed
            # agent and if the distance is within specified, add the agent to
            # the publishing message
            for agent2 in data:
                friend_position = agent2.pose.pose.position
                distance = get_distance(agent_position, friend_position)
                if distance > 0 and distance <= self.search_radius:
                    # Calculate relative position
                    rel_pos = deepcopy(agent2)
                    rel_pos.pose.pose.position.x = friend_position.x - agent_position.x
                    rel_pos.pose.pose.position.y = friend_position.y - agent_position.y
                    nearest_agents.array.append(rel_pos)

            # Publish the relative positions message
            self.nearest[key].publish(nearest_agents)

            # Initialize the wall and obstacle positions message
            avoids = PoseArray()
            avoids.header.stamp = time

            # Positions of walls and obstacles are represented as value 100 in
            # the list `self.map`. First, find the position of the observed
            # agent in this list in form of an index pair. Then search the list
            # in specified search radius and return actual positions of the
            # walls and other obstacles.
            or_col, or_row = self.pos_to_index(agent_position.x, agent_position.y)
            # if key == 'sphero_0': print (or_col, or_row)
            col_range = range(max(0, or_col - self.r), min(self.map_width, or_col + self.r + 1))
            row_range = range(max(0, or_row - self.r), min(self.map_height, or_row + self.r + 1))
            for row in row_range:
                for col in col_range:
                    # if key == 'sphero_0': print(self.map[row][col]/100, end=' ')
                    # Check only elements within radius
                    # Search obstacles in a circle instead of a square
                    if (pow(row - or_row, 2) + pow(col - or_col, 2)) <= pow(self.r, 2):
                        if self.map[row][col] == 100:
                            x, y = self.index_to_pos(row, col)
                            obst = Pose()
                            obst.position.x = x - agent_position.x
                            obst.position.y = y - agent_position.y
                            avoids.poses.append(obst)
                # if key == 'sphero_0': print('\n')

            # Publish the wall and obstacle positions message
            self.avoid[key].publish(avoids)

    def __init__(self):
        """Create subscribers and publishers."""

        # Get the number of agents
        self.num_agents = rospy.get_param("/num_of_robots")
        robot_name = rospy.get_param("~robot_name")

        # Create publishers for commands
        pub_keys = [robot_name + '_{}'.format(i) for i in range(self.num_agents)]

        # Publisher for locations of nearest agents
        self.nearest = dict.fromkeys(pub_keys)
        for key in self.nearest.keys():
            self.nearest[key] = rospy.Publisher('/' + key + '/nearest', OdometryArray, queue_size=1)

        # Publisher for locations of walls and obstacles
        self.avoid = dict.fromkeys(pub_keys)
        for key in self.avoid.keys():
            self.avoid[key] = rospy.Publisher('/' + key + '/avoid', PoseArray, queue_size=1)

        # Create subscribers
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback, queue_size=1)
        rospy.sleep(0.5)  # Wait for first map_callback to finish
        rospy.Subscriber('/dyn_reconf/parameter_updates', Config, self.param_callback, queue_size=1)
        self.param_callback(None)

        topic_name = '/' + robot_name + '_{}/odom'
        subs = [mf.Subscriber(topic_name.format(i), Odometry) for i in range(self.num_agents)]
        self.ts = mf.ApproximateTimeSynchronizer(subs, 10, 0.1)
        self.ts.registerCallback(self.robot_callback)

        # Keep program from exiting
        rospy.spin()


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('NearestSearch')

    # Go to class functions that do all the heavy lifting.
    # Do error checking.
    try:
        ns = NearestSearch()
    except rospy.ROSInterruptException:
        pass
