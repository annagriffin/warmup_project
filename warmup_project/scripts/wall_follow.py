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
        self.velocity.linear.x = 0.15
        self.velocity.angular.z = 0
        self.k = 0.6

    def callback(self, msg):
        left_error =  msg.ranges[60] - msg.ranges[120]
        print("60 degree:", msg.ranges[60])
        print("120 degree:", msg.ranges[120])
        print("left error: %s" %left_error)
        #back_error = msg.ranges[135] - msg.ranges[225]
        right_error = msg.ranges[240] - msg.ranges[300]
        print("right error: %s" %right_error)
        #front_error = msg.ranges[45] - msg.ranges[315]
        #errors = [left_error, back_error, right_error, front_error]
        errors = [left_error, right_error]
        side_str = ["left", "right"]

        # Find the side with minimum difference
        min_error = float("inf")
        min_error_side = None
        for i in range(2):
            if (not math.isnan(errors[i])) and (abs(errors[i]) < abs(min_error)):
                min_error = errors[i]
                min_error_side = side_str[i]

        # errors_absolute =  [abs(error) for error in errors]
        # errors_absolute_min = min(errors_absolute)
        # error_absolute_min_index = errors_absolute.index(errors_absolute_min)
        # min_error = errors[error_absolute_min_index]
        
        print("min_error: %s" % min_error)
        print("side: %s" %min_error_side)

        if (abs(min_error) < 0.5e-10) or min_error == float("inf"):
            self.velocity.angular.z = 0
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


           
            

            





