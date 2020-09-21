#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollowNode(object):
    def __init__(self):
        rospy.init_node('my_wall_follow_node')
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.velocity = Twist()
        self.velocity.linear.x = 0.5
        self.velocity.angular.z = 0
        self.k = 0.5



    def callback(self, msg):
        left_error = msg.ranges[45] - msg.ranges[135]
        bottom_error = msg.ranges[135] - msg.ranges[45]
        right_error = msg.ranges[135] - msg.ranges[315]
        front_error = msg.ranges[45] - msg.ranges[315]
        errors = [left_error, bottom_error, righ_error, front_error]
        min_error = min(errors)
        self.velocity.angular.z = k*min_error

    def run(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.velocity)
            r.sleep()
    
if __name__ == '__main__':
    node = WallFollowNode()
    node.run()


           
            

            





