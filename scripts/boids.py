#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function

import rospy
from geometry_msgs.msg import Twist
from util.py import Vector2

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


class Boid():
    """
    An implementation of Craig Reynolds' flocking rules and boid objects.

    Each boid (bird-oid object) maneuvers based on the positions and velocities
    of its nearby flockmates. Computation is based on three components:
    1) alignment: steer towards the average heading of local flockmates
    2) cohesion: steer to move toward the average position of local flockmates
    3) separation: steer to avoid crowding local flockmates.
    Additionally, 4th component, avoid, is implemented where boids steer away
    from obstacles in their search radius.

    Each component yields a force on boid. Total force then gives the
    acceleration which is multiplied by time and added to boid's velocity.
    Force and velocity are limited to specified amount.

    Attributes:
        position (Vector2): Boid's position
        velocity (Vector2): Boid's velocity
        mass (Vector2): Boid's mass
        alignment_factor (double): Weight for alignment component
        cohesion_factor (double): Weight for cohesion component
        separation_factor (double): Weight for separation component
        avoid_factor (double): Weight for obstacle avoiding component
        max_speed (double): Velocity upper limit
        max_force (double): Force upper limit
        friction (double): Constant friction force
        crowd_radius (double): Radius to avoid crowding
        search_radius (double): Boid's sensing radius

    Methods:
        update_parameters(self): Save parameters in class variables
        compute_alignment(self, nearest_agents): Return alignment component
        compute_cohesion(self, nearest_agents): Return cohesion component
        compute_separation(self, nearest_agents): Return separation component
        compute_avoids(self, avoids): Return avoid component
        compute_velocity(self, my_agent, nearest_agents, avoids):
            Compute total velocity based on all components
    """

    def __init__(self):
        """Create an empty boid and update parameters."""
        self.position = Vector2()
        self.velocity = Vector2()
        self.update_parameters()
        self.mass = 0.18  # Mass of Sphero robot in kilograms

    def update_parameters(self):
        """Save Reynolds controller parameters in class variables."""
        self.alignment_factor = rospy.get_param('/dyn_reconf/alignment_factor')
        self.cohesion_factor = rospy.get_param('/dyn_reconf/cohesion_factor')
        self.separation_factor = rospy.get_param('/dyn_reconf/separation_factor')
        self.avoid_factor = rospy.get_param('/dyn_reconf/avoid_factor')
        self.max_speed = rospy.get_param('/dyn_reconf/max_speed')
        self.max_force = rospy.get_param('/dyn_reconf/max_force')
        self.friction = rospy.get_param('/dyn_reconf/friction')
        self.crowd_radius = rospy.get_param('/dyn_reconf/crowd_radius')
        self.search_radius = rospy.get_param('/dyn_reconf/search_radius')

        rospy.loginfo(rospy.get_caller_id() + " -> Parameters updated")
        if DEBUG:
            print('alignment_factor:  ', self.alignment_factor)
            print('cohesion_factor:  ', self.cohesion_factor)
            print('separation_factor:  ', self.separation_factor)
            print('avoid_factor:  ', self.avoid_factor)
            print('max_speed:  ', self.max_speed)
            print('max_force:  ', self.max_force)
            print('friction:  ', self.friction)
            print('crowd_radius:  ', self.crowd_radius)
            print('search_radius:  ', self.search_radius)

    def compute_alignment(self, nearest_agents):
        """Return alignment component."""
        mean_velocity = Vector2()
        steer = Vector2()
        # Find mean velocity of neighboring agents
        for agent in nearest_agents:
            agent_velocity = get_agent_velocity(agent)
            mean_velocity += agent_velocity

        # Steer toward calculated mean velocity
        if nearest_agents:
            mean_velocity.set_mag(self.max_speed)
            steer = mean_velocity - self.velocity
            steer.limit(self.max_force)
        return steer

    def compute_cohesion(self, nearest_agents):
        """Return cohesion component."""
        mean_position = Vector2()
        steer = Vector2()
        # Find mean position of neighboring agents
        for agent in nearest_agents:
            agent_position = get_agent_position(agent)
            mean_position += agent_position
        direction = mean_position / len(nearest_agents)

        # Steer toward calculated mean position
        # Force is proportional to agents distance from mean
        d = direction.norm()
        if d > 0:
            direction.set_mag(self.max_speed * (d / self.search_radius))
            steer = direction - self.velocity
            steer.limit(self.max_force)
        return steer

    def compute_separation(self, nearest_agents):
        """Return separation component."""
        direction = Vector2()
        steer = Vector2()
        count = 0
        # Find mean position of neighboring agents
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

        # Steer away from calculated mean position
        if count:
            direction.set_mag(self.max_speed)
            steer = direction - self.velocity
            steer.limit(self.max_force)
        return steer

    def compute_avoids(self, avoids):
        """Return avoid component."""
        direction = Vector2()
        steer = Vector2()
        # Find mean position of obstacles
        for obst in avoids:
            obst_position = get_obst_position(obst)
            d = obst_position.norm()
            obst_position *= -1  # Make vector point away from obstacle
            obst_position.normalize()  # Normalize to get only direction
            # Vector's magnitude is reciprocal to distance between agents
            obst_position /= d
            direction += obst_position

        # Steer away from calculated mean position
        if avoids:
            direction.set_mag(self.max_speed)
            steer = direction - self.velocity
            steer.limit(self.max_force)
        return steer

    def compute_velocity(self, my_agent, nearest_agents, avoids):
        """Compute total velocity based on all components."""
        force = Vector2()
        self.velocity = get_agent_velocity(my_agent)

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

        # Add components together and limit the output
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
