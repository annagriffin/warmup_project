#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollowNode(object):
    def __init__(self):
        rospy.init_node('my_wall_follow_node')
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        self.velocity = Twist()
        self.velocity.linear.x = 0.3
        self.velocity.angular.z = 0
        self.k = 1.3 # proportional control paramter for counter clockwise
        self.k_cw = 1 # proportional control parameter for clockwise turn
        self.angle_LS1 = 70
        self.angle_LS2 = 110
        self.angle_RS1 = 240
        self.angle_RS2 = 300
        self.angle_TS1 = 330
        self.angle_TS2 = 30

    def callback(self, msg):
        laser_scan = msg.ranges

        left_error = laser_scan[self.angle_LS1] - laser_scan[self.angle_LS2]
        left_scan_avg = (laser_scan[self.angle_LS1] + laser_scan[self.angle_LS2])/2

        right_error = laser_scan[self.angle_RS1] - laser_scan[self.angle_RS2]
        right_scan_avg = (laser_scan[self.angle_RS1] + laser_scan[self.angle_RS2])/2

        top_error = laser_scan[self.angle_TS2] - laser_scan[self.angle_TS1]
        top_scan_avg = (laser_scan[self.angle_TS2] + laser_scan[self.angle_TS1])/2
        
        errors = [left_error, top_error, right_error]
        scan_avgs = [left_scan_avg, top_scan_avg, right_scan_avg]
        side_str = ["left", "top", "right"]

        # Find the wall closet to robot
        min_error = float("inf")
        min_scan_avg = float("inf")
        min_error_side = None
        for i in range(len(errors)):
            if (not math.isnan(errors[i])) and (scan_avgs[i] < min_scan_avg):
                min_error = errors[i]
                min_error_side = side_str[i]
        
        print("min_error: %s" % min_error)
        print("side: %s" %min_error_side)

        if (min_error == float("inf") or abs(min_error) < 1e-2):
            self.velocity.angular.z = 0
        elif min_error < 0:
            self.velocity.angular.z = self.k_cw*min_error
        else:
            self.velocity.angular.z = self.k*min_error

    def run(self):
        r = rospy.Rate(2)
        while not rospy.is_shutdown():
            print(self.velocity)
            self.vel_pub.publish(self.velocity)
            r.sleep()


if __name__ == '__main__':
    node = WallFollowNode()
    node.run()


           
            






