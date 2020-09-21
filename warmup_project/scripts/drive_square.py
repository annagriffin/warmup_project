#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rospy
import math

class DriveSquareNode(object):

    def __init__(self):
        rospy.init_node("drive_square")
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.velocity = Twist()

    def move(self):
        self.velocity.linear.x = 1
        self.velocity.angular.z = 0
        print(self.velocity)
        self.pub.publish(self.velocity)

    def turn(self):
        # print("TURN")
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0.5
        self.pub.publish(self.velocity)

    def run(self):
        angle = math.pi/2*2
        print("start")
        start = rospy.Time.now()
        while(rospy.Time.now() - start <= rospy.Duration.from_sec(10)):
            self.move()
        print("end move")
        start = rospy.Time.now()
        # while(rospy.Time.now() - start <= rospy.Duration.from_sec(angle)):
        #     self.turn()
        # rospy.Timer(rospy.Duration(10.0), self.move())
        # rospy.Timer(rospy.Duration(angle), self.turn())
        # rospy.Timer(rospy.Duration(10), self.move())
        # rospy.Timer(rospy.Duration(angle), self.turn())
        # rospy.Timer(rospy.Duration(10), self.move())
        # rospy.Timer(rospy.Duration(angle), self.turn())
        # rospy.Timer(rospy.Duration(10), self.move())
        

if __name__== "__main__":
    node = DriveSquareNode()
    node.run()
        