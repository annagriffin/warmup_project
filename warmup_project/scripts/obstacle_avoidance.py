#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class ObstacleAvoidanceNode(object):
    def __init__(self):
        rospy.init_node('my_wall_follow_node')
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.velocity = Twist()
        self.delta_x_obstacle = []
        self.delta_y_obstable = []
        self.circle_of_influence = 1
        self.beta = 1 # strength of obstacle field
        self.alpha = 5 # strength of goal field
        self.goal_distance = 1.5 # how far away the goal in front is
        self.k_velocity = 0.05
        self.k_angle = 0.8

    def callback(self, msg):
        laser_scan = msg.ranges
        # iterate through front laser scans
        for i in range(-90,91,2):
            i = i%360
            d = laser_scan[i]
            if d != float("inf"):
                angle_rad = math.radians(i)
                self.compute_obstacle_potential_field(d, angle_rad)

        # Adding all vectors from repulsive obstacle and attractive goal
        goal_delta_x, goal_delta_y = self.compute_goal_potential_field()
        delta_x_total = sum(self.delta_x_obstacle) + goal_delta_x
        delta_y_total = sum(self.delta_y_obstable) + goal_delta_y

        # publish desired velocity
        self.velocity.linear.x = self.k_velocity*math.sqrt(delta_x_total**2 + delta_y_total**2)
        self.velocity.angular.z = self.k_angle*math.tanh(delta_y_total/delta_x_total)
        self.vel_pub.publish(self.velocity)

        # reset potential fields
        self.delta_x_obstacle = []
        self.delta_y_obstable = []

        print("goal delta x, y:", goal_delta_x, ", ", goal_delta_y)
        print("x_delta_total:", delta_x_total)
        print("y_delta_total:", delta_y_total)
        print(self.velocity)

    def compute_obstacle_potential_field(self, distance, angle):
        if distance <= self.circle_of_influence:
            delta_x = - self.beta*(self.circle_of_influence-distance)*math.cos(angle)
            delta_y = - self.beta*(self.circle_of_influence-distance)*math.sin(angle)
            self.delta_x_obstacle.append(delta_x)
            self.delta_y_obstable.append(delta_y)

    def compute_goal_potential_field(self):
        delta_x = self.alpha*self.goal_distance*math.cos(0)
        delta_y = self.alpha*self.goal_distance*math.sin(0)
        return delta_x, delta_y

    def run(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            r.sleep()

if __name__ == '__main__':
    node = ObstacleAvoidanceNode()
    node.run()










