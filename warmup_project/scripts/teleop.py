#!/usr/bin/env python3

import tty
import select
import sys
import termios
import rospy
from geometry_msgs.msg import Twist

class TeletopKeyboard(object):
    def __init__(self):
        rospy.init_node('my_teleop_twist_keyboard')
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.theta = 0.0
        self.speed = 0.5 # linear speed
        self.turn = 1.0 # angular speed
        self.key = None
        self.settings = termios.tcgetattr(sys.stdin)


        self.updated = Twist() # updated velocity

        self.moveBindings = { #(x,y,z,theta)
            'i':(1,0,0,0), # Forward
            ',':(-1,0,0,0), # Backward
            'j':(0,0,0,1), # Rotate left in space (CCW)
            'l':(0,0,0,-1), # Rotate right in space (CW)
            'u':(1,0,0,1), # Forward and rotate left (CCW)
            '.':(-1,0,0,1), # Backward and rotate left (CCW)
            'o':(1,0,0,-1), # Forward and rotate right (CW)
            'm':(-1,0,0,-1) # Backward and rotate right (CW)
            'k':(0,0,0,0) # stop
            }

        self.speedBindings = { # (linear, angular)
            'q': (1.1, 1.1),  # increase speed (linear and angular)
            'z': (0.9, 0.9),  # decrease speed (linear and angular)
            'w': (1.1, 1),    # increse linear speed
            'x': (0.9, 1),    # decrease linear speed
            'e': (1, 1.1),    # increase angular speed
            'c': (1, .9)      # decrease angular speed
            }

    def update(self, key):
        
        if key in self.moveBindings.keys():
            self.x = self.moveBindings[key][0]
            self.y = self.moveBindings[key][1]
            self.z = self.moveBindings[key][2]
            self.th = self.moveBindings[key][3]
        elif key in self.speedBindings.keys():
            self.speed = self.speed * self.speedBindings[key][0] 
            self.turn = self.turn * self.speedBindings[key][1] 
        elif key == '\x03':
            return
        else:
            self.x = 0.0
            self.y = 0.0
            self.z = 0.0
            self.theta = 0.0

        # linear velocity updates
        self.updated.linear.x = self.x * self.speed
        self.updated.linear.y = self.y * self.speed
        self.updated.linear.z = self.z * self.speed

        # angular velocity updates
        self.updated.angular.x = 0
        self.updated.angular.y = 0
        self.updated.angular.z = self.theta * self.turn


        self.pub.publish(self.updated)
        print("currently:\tspeed %s\tturn %s " % (self.speed,self.turn))
        #print("currently:\t x: %s\t y: %s\t z: %s\t theta: %s\t" % (self.updated.linear.x, self.updated.linear.y, self.updated.linear.z, self.updated.angular.z))
        print("Updated velocity: %s" %self.updated)

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def run(self):
        
        while self.key != '\x03':
            self.key = self.getKey()
            print("key: " + self.key)
            self.update(self.key)

if __name__== "__main__":
    node = TeletopKeyboard()
    node.run()