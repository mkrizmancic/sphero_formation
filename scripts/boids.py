from __future__ import print_function

import math
import rospy
from geometry_msgs.msg import Twist

DEBUG = False


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

    Supports simple addition, substraction, multiplication, division and
    normalization, as well as getting norm and angle of the vector and
    setting limit and magnitude.
    """

    def __init__(self, x=0, y=0):
        self.x = x
        self.y = y

    def __add__(self, other):
        if isinstance(other, self.__class__):
            return Vector2(self.x + other.x, self.y + other.y)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x + other, self.y + other)

    def __sub__(self, other):
        if isinstance(other, self.__class__):
            return Vector2(self.x - other.x, self.y - other.y)
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x - other, self.y - other)

    def __div__(self, other):
        if isinstance(other, self.__class__):
            raise ValueError("Cannot divide two vectors!")
        elif isinstance(other, int) or isinstance(other, float):
            if other != 0:
                return Vector2(self.x / other, self.y / other)
            else:
                return Vector2()

    def __mul__(self, other):
        if isinstance(other, self.__class__):
            raise NotImplementedError("Multiplying vectors is not implemented!")
        elif isinstance(other, int) or isinstance(other, float):
            return Vector2(self.x * other, self.y * other)

    def __rmul__(self, other):
        return self.__mul__(other)

    def __repr__(self):
        return "({: 6.1f}, {: .5f})".format(self.arg(), self.norm())
        # return "({: .3f}, {: .3f})".format(self.x, self.y)

    def norm(self):
        """Return the norm of the vector."""
        return math.sqrt(pow(self.x, 2) + pow(self.y, 2))

    def arg(self):
        """Return the angle of the vector."""
        return math.degrees(math.atan2(self.y, self.x))

    def normalize(self, ret=False):
        """Normalize the vector."""
        d = self.norm()
        if d:
            if not ret:
                self.x /= d
                self.y /= d
            else:
                return Vector2(self.x / d, self.y / d)

    def limit(self, value):
        """Limit vector's maximum magnitude to given value."""
        if self.norm() > value:
            self.setMag(value)

    def setMag(self, value):
        """Set vector's magnitude without changing direction."""
        self.normalize()
        self.x *= value
        self.y *= value


class Boid():
    """TODO: MISSING DOCSTRING!"""

    def __init__(self):
        """Create an empty boid and update parameters."""
        self.position = Vector2()
        self.velocity = Vector2()
        self.update_parameters()
        self.mass = 0.18  # Mass of Sphero robot in kilograms

    def update_parameters(self):
        """Save Reynolds controller parametars in class variables."""
        self.alignment_factor = rospy.get_param('/dyn_reconf/alignment_factor')
        self.cohesion_factor = rospy.get_param('/dyn_reconf/cohesion_factor')
        self.separation_factor = rospy.get_param('/dyn_reconf/separation_factor')
        self.avoid_factor = rospy.get_param('/dyn_reconf/avoid_factor')
        self.max_speed = rospy.get_param('/dyn_reconf/max_speed')
        self.max_force = rospy.get_param('/dyn_reconf/max_force')
        self.friction = rospy.get_param('/dyn_reconf/friction')
        self.crowd_radius = rospy.get_param('/dyn_reconf/crowd_radius')
        self.search_radius = rospy.get_param('/dyn_reconf/search_radius')

    def compute_alignment(self, nearest_agents):
        """Return alignment component."""
        mean_velocity = Vector2()
        steer = Vector2()
        for agent in nearest_agents:
            agent_velocity = get_agent_velocity(agent)
            mean_velocity += agent_velocity

        if nearest_agents:
            mean_velocity.setMag(self.max_speed)
            steer = mean_velocity - self.velocity
            steer.limit(self.max_force)
        return steer

    def compute_cohesion(self, nearest_agents):
        """Return cohesion component."""
        mean_position = Vector2()
        steer = Vector2()
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            mean_position += agent_position
        direction = mean_position / len(nearest_agents)

        d = direction.norm()
        if d > 0:
            direction.setMag(self.max_speed * (d / self.search_radius))
            steer = direction - self.velocity
            steer.limit(self.max_force)
        return steer

    def compute_separation(self, nearest_agents):
        """Return separation component."""
        direction = Vector2()
        steer = Vector2()
        count = 0
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            if agent_position.norm() < self.crowd_radius:
                count += 1
                d = agent_position.norm()
                agent_position *= -1  # Make vector point away from other agent
                agent_position.normalize()  # Normalize to get only direction
                # Vector's magnitude is reciprocal to distance between agents
                agent_position /= d
                direction += agent_position

        if count:
            direction.setMag(self.max_speed)
            steer = direction - self.velocity
            steer.limit(self.max_force)
        return steer

    def compute_avoids(self, avoids):
        """Return avoid component."""
        direction = Vector2()
        steer = Vector2()
        d = 10000000
        for obst in avoids:
            obst_position = get_obst_position(obst)
            d = min(d, obst_position.norm())
            obst_position *= -1
            obst_position.normalize()
            obst_position /= d
            direction += obst_position

        if avoids:
            direction.setMag(self.max_speed)
            steer = direction - self.velocity
            steer.limit(self.max_force)
        return steer

    def compute_velocity(self, my_agent, nearest_agents, avoids):
        """Compute total velocity based on all components."""
        force = Vector2()
        self.velocity = get_agent_velocity(my_agent)

        # Update all dynamic parameters
        self.update_parameters()

        # Compute all the components
        alignment = self.compute_alignment(nearest_agents)
        cohesion = self.compute_cohesion(nearest_agents)
        separation = self.compute_separation(nearest_agents)
        avoid = self.compute_avoids(avoids)

        if DEBUG:
            print("alignment:    ", alignment)
            print("cohesion:     ", cohesion)
            print("separation:   ", separation)
            print("avoid:        ", avoid)

        # Add componets together and limit the output
        force += alignment * self.alignment_factor
        force += cohesion * self.cohesion_factor
        force += separation * self.separation_factor
        force += avoid * self.avoid_factor
        force.limit(self.max_force)

        # If agent is moving, apply constant friction force
        if self.velocity.norm() > 0:
            force += self.friction * -1 * self.velocity.normalize(ret=True)

        acceleration = force / self.mass

        # Calculate total velocity (delta_velocity = acceleration * delta_time)
        self.velocity += acceleration / 10
        self.velocity.limit(self.max_speed)

        if DEBUG:
            print("force:        ", force)
            print("acceleration: ", acceleration)
            print("velocity:     ", self.velocity)
            print()

        # Return the the velocity as Twist message
        vel = Twist()
        vel.linear.x = self.velocity.x
        vel.linear.y = self.velocity.y
        return vel
