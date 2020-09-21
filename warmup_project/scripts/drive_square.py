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
        self.velocity.angular.z = 1
        self.pub.publish(self.velocity)

    def run(self):
        angle = math.pi/2

        # First Line
        start = rospy.Time.now()
        while(rospy.Time.now() - start <= rospy.Duration.from_sec(1)):
            self.move()
        start = rospy.Time.now()
        while(rospy.Time.now() - start <= rospy.Duration.from_sec(angle)):
            self.turn()

        # Second Line
        start = rospy.Time.now()
        while(rospy.Time.now() - start <= rospy.Duration.from_sec(1)):
            self.move()
        start = rospy.Time.now()
        while(rospy.Time.now() - start <= rospy.Duration.from_sec(angle)):
            self.turn()

        # Third Line
        start = rospy.Time.now()
        while(rospy.Time.now() - start <= rospy.Duration.from_sec(1)):
            self.move()

        start = rospy.Time.now()
        while(rospy.Time.now() - start <= rospy.Duration.from_sec(angle)):
            self.turn()

        # Last Line
        start = rospy.Time.now()
        while(rospy.Time.now() - start <= rospy.Duration.from_sec(1)):
            self.move()

        # Stop
        self.velocity.linear.x = 0
        self.velocity.angular.z = 0
        self.pub.publish(self.velocity)
        

if __name__== "__main__":
    node = DriveSquareNode()
    node.run()
        