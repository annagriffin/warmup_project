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
        self.k = 3
        self.k_cw = 3.25
        self.angle_LS1 = 70
        self.angle_LS2 = 110
        self.angle_BS1 = 160
        self.angle_BS2 = 200
        self.angle_RS1 = 250
        self.angle_RS2 = 290
        self.angle_TS1 = 340
        self.angle_TS2 = 20 

    def callback(self, msg):
        laser_scan = msg.ranges
        left_error = laser_scan[self.angle_LS1] - laser_scan[self.angle_LS2]
        back_error = laser_scan[self.angle_BS2] - laser_scan[self.angle_BS1]
        right_error = laser_scan[self.angle_RS1] - laser_scan[self.angle_RS2]
        top_error = laser_scan[self.angle_TS2] - laser_scan[self.angle_TS1]

        print("RS1 degree:", laser_scan[self.angle_RS1])
        print("RS2 degree:", laser_scan[self.angle_RS2])
        print("right error: %s" %right_error)
        print("BS1 degree:", laser_scan[self.angle_BS1])
        print("BS2 degree:", laser_scan[self.angle_BS2])
        print("back error: %s" %back_error)
        print("top error: %s" %top_error)
        print("left error: %s" %left_error)
        
        errors = [left_error, top_error, right_error, back_error]
        side_str = ["left", "top", "right", "back"]

        # Find the side with minimum difference
        min_error = float("inf")
        min_error_side = None
        for i in range(len(errors)):
            if (not math.isnan(errors[i])) and (abs(errors[i]) < abs(min_error)):
                min_error = errors[i]
                min_error_side = side_str[i]
        
        print("min_error: %s" % min_error)
        print("side: %s" %min_error_side)

        if (min_error == float("inf") or abs(min_error) < 0.5e-10):
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


           
            






