from __future__ import print_function

import math
import rospy
from geometry_msgs.msg import Twist


def get_agent_velocity(agent):
    """Return agent velocity as Vector2 instance."""
    vel = Vector2()
    vel.x = agent.twist.twist.linear.x
    vel.y = agent.twist.twist.linear.y
    return vel


def get_agent_position(agent):
    """Return agent position as Vector2 instance."""
    pos = Vector2()
    pos.x = agent.pose.pose.position.x
    pos.y = agent.pose.pose.position.y
    return pos


def get_obst_position(obst):
    """Return obstacle position as Vector2 instance."""
    pos = Vector2()
    pos.x = obst.position.x
    pos.y = obst.position.y
    return pos


class Vector2():
    """
    2D vector class representation with x and y components.

    Enables simple addition, substraction, multiplication, division and
    normalization of 2D vectors.
    """

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __div__(self, other):
        if isinstance(other, self.__class__):
            pass
        elif isinstance(other, int):
            return Vector2(self.x / other, self.y / other)
        elif isinstance(other, float):
            return Vector2(self.x / other, self.y / other)

    def __mul__(self, other):
        if isinstance(other, self.__class__):
            pass
        elif isinstance(other, int):
            return Vector2(self.x * other, self.y * other)
        elif isinstance(other, float):
            return Vector2(self.x * other, self.y * other)

    def __repr__(self):
        return "({}, {})".format(self.x, self.y)

    def norm(self):
        """Return the norm of the vector."""
        return math.sqrt(pow(self.x, 2) + pow(self.y, 2))

    def normalize(self):
        """Normalize the vector."""
        d = self.norm()
        if d:
            self.x /= d
            self.y /= d


class Boid():

    def __init__(self):
        """Create an empty boid and update parametars."""
        self.position = Vector2()
        self.velocity = Vector2()
        self.update_parameters()

    def update_parameters(self):
        """Save Reynolds controller parametars in class variables."""
        self.alignment_factor = rospy.get_param('/dyn_reconf/alignment_factor')
        self.cohesion_factor = rospy.get_param('/dyn_reconf/cohesion_factor')
        self.separation_factor = rospy.get_param('/dyn_reconf/separation_factor')
        self.max_speed = rospy.get_param('/dyn_reconf/max_speed')
        self.crowd_radius = rospy.get_param('/dyn_reconf/crowd_radius')

    # Radiusi za alignment i cohesion su isti i ne treba ih zadavati jer ionako
    # primamo samo susjede koji su u definiranom radijusu iz nekog vanjskog node
    def compute_alignment(self, nearest_agents):
        """Return alignment component."""
        velocity = Vector2()
        for agent in nearest_agents:
            agent_velocity = get_agent_velocity(agent)
            velocity += agent_velocity

        velocity.normalize()
        return velocity

    def compute_cohesion(self, nearest_agents):
        """Return cohesion component."""
        direction = Vector2()
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            direction += agent_position

        direction.normalize()
        return direction

    # Radius za separation je manji
    def compute_separation(self, nearest_agents):
        """Return separation component."""
        direction = Vector2()
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            if agent_position.norm() < self.crowd_radius:
                direction += agent_position

        direction *= -1
        direction.normalize()
        return direction

    def compute_colision(self, nearest_agents):
        """Return colision component."""
        direction = Vector2()
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            if agent_position.norm() < 0.3:
                direction += agent_position

        direction *= -1
        direction.normalize()
        return direction

    def compute_avoids(self, avoids):
        """Return avoid component."""
        direction = Vector2()
        for obst in avoids:
            obst_position = get_obst_position(obst)
            # print (obst_position, end=' ')
            direction += obst_position

        direction *= -1
        direction.normalize()
        return direction

    def compute_velocity(self, nearest_agents, avoids):
        """Compute total velocity based on all components."""
        self.update_parameters()
        alignment = self.compute_alignment(nearest_agents)
        cohesion = self.compute_cohesion(nearest_agents)
        separation = self.compute_separation(nearest_agents)
        avoid = self.compute_avoids(avoids)
        # colision = self.compute_colision(nearest_agents)

        self.velocity += alignment * self.alignment_factor
        self.velocity += cohesion * self.cohesion_factor
        self.velocity += separation * self.separation_factor
        self.velocity.normalize()
        self.velocity += avoid
        self.velocity.normalize()
        self.velocity *= self.max_speed

        vel = Twist()
        vel.linear.x = self.velocity.x
        vel.linear.y = self.velocity.y
        return vel
