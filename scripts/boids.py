import math
import rospy
from geometry_msgs.msg import Twist


def get_agent_velocity(agent):
    vel = Vector2()
    vel.x = agent.twist.twist.linear.x
    vel.y = agent.twist.twist.linear.y
    return vel


def get_agent_position(agent):
    pos = Vector2()
    pos.x = agent.pose.pose.position.x
    pos.y = agent.pose.pose.position.y
    return pos


class Vector2():
    """docstring for Vector2"""

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
        return math.sqrt(pow(self.x, 2) + pow(self.y, 2))

    def normalize(self):
        d = self.norm()
        if d:
            self.x /= d
            self.y /= d


class Boid():

    def __init__(self):
        self.position = Vector2()
        self.velocity = Vector2()
        self.update_parameters()

    def update_parameters(self):
        self.alignment_factor = rospy.get_param('/dyn_reconf/alignment_factor')
        self.cohesion_factor = rospy.get_param('/dyn_reconf/cohesion_factor')
        self.separation_factor = rospy.get_param('/dyn_reconf/separation_factor')
        self.max_speed = rospy.get_param('/dyn_reconf/max_speed')
        self.crowd_radius = rospy.get_param('/dyn_reconf/crowd_radius')

    # Radiusi za alignment i cohesion su isti i ne treba ih zadavati jer ionako
    # primamo samo susjede koji su u definiranom radijusu iz nekog vanjskog node
    def compute_alignment(self, nearest_agents):
        velocity = Vector2()
        for agent in nearest_agents:
            agent_velocity = get_agent_velocity(agent)
            velocity += agent_velocity

        if nearest_agents:
            velocity /= len(nearest_agents)
            velocity.normalize()
        return velocity

    def compute_cohesion(self, nearest_agents):
        direction = Vector2()
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            direction += agent_position

        if nearest_agents:
            direction /= len(nearest_agents)
            direction.normalize()  # normalize vector (divide with its length)
        return direction

    # Radius za separation je manji
    def compute_separation(self, nearest_agents):
        direction = Vector2()
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            if agent_position.norm() < self.crowd_radius:
                direction += agent_position

        if nearest_agents:
            direction /= len(nearest_agents)
            direction *= -1  # negate the vector
            direction.normalize()  # normalize vector (divide with its length)
        return direction

    # TODO: dodati izbjegavanje prepreka

    def compute_velocity(self, nearest_agents):
        self.update_parameters()
        alignment = self.compute_alignment(nearest_agents)
        cohesion = self.compute_cohesion(nearest_agents)
        separation = self.compute_separation(nearest_agents)
        print(alignment)
        print(cohesion)
        print(separation)
        print

        print(self.velocity)
        self.velocity += alignment * self.alignment_factor
        self.velocity += cohesion * self.cohesion_factor
        self.velocity += separation * self.separation_factor
        self.velocity.normalize()
        self.velocity *= self.max_speed
        print(self.velocity)
        print

        vel = Twist()
        vel.linear.x = self.velocity.x
        vel.linear.y = self.velocity.y
        return vel
