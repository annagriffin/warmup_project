#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rospy
import math


class PersonFollowerNode(object):

    def __init__(self):
        rospy.init_node("person_follower")
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.velocity = Twist()
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.callback)
        


        def callback(self, msg):
            laser_scan = mgs.ranges
            # points 90 degress to left and 90 degress to right
            # of the center of the neato
            points_in_front = laser_scan[:90] + laser_scan[270:]

            

        def get_x_position(self, points):

            distances = []
            while i < len(points) and (i < 90 or i > 270)
                
                if not math.isinf(points[i]) and not math.isnan(points[i]):
                    
                    distances.append(p * math.sin(i))
            
            return sum(distances) / len(distances)
            
        def get_y_position(self, points):

            distances = []
            while i < len(points) and (i < 90 or i > 270)
                
                if not math.isinf(points[i]) and not math.isnan(points[i]):
                    
                    distances.append(p * math.cos(i))
            
            return sum(distances) / len(distances)

        def adjust(self, range):

            x_cm = self.get_x_position(range)
            y_cm = self.get_y_position(range)

            angle_diff = math.tanh(x_cm / y_cm)


            # turn


        def run(self):
            r = rospy.Rate(2)
            while not rospy.is_shutdown():
                print(self.velocity)
                self.vel_pub.publish(self.velocity)
                r.sleep()


if __name__ == '__main__':
    node = PersonFollowerNode()
    node.run()