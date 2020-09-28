#!/usr/bin/env python3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rospy
import math


class PersonFollowerNode(object):

    def __init__(self):
        rospy.init_node("person_follower")
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.velocity = Twist()
        self.linear_speed = 0.8
        self.COM_x = 0
        self.COM_y = 0
        self.k = 1.5

    def callback(self, msg):
        laser_scan = msg.ranges
        # points 90 degress to left and 90 degress to right
        # of the center of the neato
        self.compute_com_x(laser_scan)
        self.compute_com_y(laser_scan)
        self.adjust()

    def compute_com_x(self, scans):
        distances = []
        for i in range(-90, 91, 1):
            i = i % 360
            if not math.isinf(scans[i]):
                distances.append(scans[i] * math.sin(i))
        if len(distances) > 0:
            self.COM_x = sum(distances) / len(distances)
        else:
            self.COM_x = 0
        print("x com:", self.COM_x)

    def compute_com_y(self, scans):
        distances = []
        for i in range(-90, 91, 1):
            i = i % 360
            if not math.isinf(scans[i]):
                distances.append(scans[i] * math.cos(i))
        if len(distances) > 0:
            self.COM_y = sum(distances) / len(distances)
        else:
            self.COM_y = 0
        print("y com:", self.COM_y)

    def adjust(self):
        COM_distance = math.sqrt(self.COM_x**2 + self.COM_y**2)
        if self.COM_y != 0 and COM_distance > 0.03:
            self.velocity.angular.z = self.k * math.tanh(self.COM_x / abs(self.COM_y))
            r = self.k * math.tanh(self.COM_x / abs(self.COM_y)) * 57.95
            print(round(r))
            self.velocity.linear.x = self.linear_speed
            self.vel_pub.publish(self.velocity)
        else:
            self.velocity.angular.z = 0
            self.velocity.linear.x = 0
            self.vel_pub.publish(self.velocity)

    def run(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            r.sleep()


if __name__ == '__main__':
    node = PersonFollowerNode()
    node.run()
