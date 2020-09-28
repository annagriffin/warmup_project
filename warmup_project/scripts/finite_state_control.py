#!/usr/bin/env python3
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import rospy
import math

class FiniteStateControlNode(object):

    def __init__(self):
        rospy.init_node("finite_state_controler_node")
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.process_scan)
        
        # initialize velocity
        self.velocity = Twist()

        # wall follower parameter
        self.k_cc = 1.35 # proportional control paramter for counter clockwise
        self.k_cw = 1.3 # proportional control parameter for clockwise turn
        self.COM_x = 0
        self.COM_y = 0
        self.angle_LS1 = 70
        self.angle_LS2 = 110
        self.angle_RS1 = 245
        self.angle_RS2 = 295
        self.linear_speed_wall = 0.3
        self.min_error = float("inf")

        # person follow parameter
        self.linear_speed_person = 0.025
        self.k_obj = 2.5 # proportional control parameter for person follower
        self.COM_distance_threshold = 0.05
        self.view_angle = [0,90]

        # finite state machine parameter
        self.object_in_view = False
        self.laser_scan = []
        self.state = self.wall_follow

    def process_scan(self, msg):
        self.laser_scan = msg.ranges
        self.object_in_view = False
        # check if there is still person in the view
        counter = 0
        for i in range(self.view_angle[0], self.view_angle[1], 1):
            i = i%360
            if not math.isinf(self.laser_scan[i]):
                counter += 1

        if counter >= 28:
            self.object_in_view = True

        if self.object_in_view:
            self.person_follow_process_scan()
        else:
            self.wall_follow_process_scan()
        
    def person_follow_process_scan(self):
        self.compute_com_x(self.laser_scan)
        self.compute_com_y(self.laser_scan)
        self.adjust()

    def person_follow(self):
        self.vel_pub.publish(self.velocity)
        print("person follow velocity: ", "\n", self.velocity)
        if not self.object_in_view:
            return self.wall_follow
        else:
            return self.person_follow

    def wall_follow_process_scan(self):
        left_error = self.laser_scan[self.angle_LS1] - self.laser_scan[self.angle_LS2]
        left_scan_avg = (self.laser_scan[self.angle_LS1] + self.laser_scan[self.angle_LS2])/2

        right_error = self.laser_scan[self.angle_RS1] - self.laser_scan[self.angle_RS2]
        right_scan_avg = (self.laser_scan[self.angle_RS1] + self.laser_scan[self.angle_RS2])/2

        errors = [left_error, right_error]
        scan_avgs = [left_scan_avg, right_scan_avg]
        side_str = ["left", "right"]

        # Find the wall closet to robot
        self.min_error = float("inf")
        min_scan_avg = float("inf")
        side = None
        for i in range(len(errors)):
            if (not math.isnan(errors[i])) and (scan_avgs[i] < min_scan_avg):
                self.min_error = errors[i]
                side = side_str[i]
        
        print("min_error: %s" % self.min_error)
        print("side: %s" %side)
    
    def wall_follow(self):
        print("wall follow?", not self.object_in_view)
        self.velocity.linear.x = self.linear_speed_wall
        if (self.min_error == float("inf") or abs(self.min_error) < 1e-2):
            self.velocity.angular.z = 0
        elif self.min_error < 0:
            self.velocity.angular.z = self.k_cw*self.min_error
        else:
            self.velocity.angular.z = self.k_cc*self.min_error

        self.vel_pub.publish(self.velocity)
        print("wall follow velocity: ", "\n", self.velocity)

        # check if need to switch to person follow
        if self.object_in_view:
            return self.person_follow
        else:
            return self.wall_follow
    
    def compute_com_x(self, scans):
        distances = []
        for i in range(self.view_angle[0], self.view_angle[1], 1):
            i = i % 360
            if not math.isinf(scans[i]):
                distances.append(scans[i] * math.sin(i))
        if len(distances) > 0:
            self.COM_x = sum(distances) / len(distances)
        else:
            self.COM_x = 0
        # print("x com:", self.COM_x)

    def compute_com_y(self, scans):
        distances = []
        for i in range(self.view_angle[0], self.view_angle[1], 1):
            i = i % 360
            if not math.isinf(scans[i]):
                distances.append(scans[i] * math.cos(i))
        if len(distances) > 0:
            self.COM_y = sum(distances) / len(distances)
        else:
            self.COM_y = 0
        # print("y com:", self.COM_y)

    def adjust(self):
        # pythagorean theorem to find distance to COM point
        COM_distance = math.sqrt(self.COM_x**2 + self.COM_y**2)

        # set stopping threshold if it gets close
        if self.COM_y != 0 and COM_distance > self.COM_distance_threshold:
            self.velocity.angular.z = self.k_obj * math.tanh(self.COM_x / abs(self.COM_y))
            self.velocity.linear.x = self.linear_speed_person
        else:
            self.velocity.angular.z = 0
            self.velocity.linear.x = 0

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            self.state = self.state()
            r.sleep()
        
    # def callback(self, msg):

    #     laser_scan = msg.ranges


    #     # if one or more values are inf, the sum will be inf
    #     top_error = laser_scan[:self.angle_TS2] + laser_scan[self.angle_TS1:]
    #     top_error_sum = np.sum(top_error)
    #     print("error sum: " + str(top_error_sum))
    #     top_error_is_inf = np.isinf(top_error_sum)

    #     # nothing detected infront of robot
    #     if np.isinf(top_error_sum):

    #         left_error = laser_scan[self.angle_LS1] - laser_scan[self.angle_LS2]
    #         left_scan_avg = (laser_scan[self.angle_LS1] + laser_scan[self.angle_LS2])/2

    #         right_error = laser_scan[self.angle_RS1] - laser_scan[self.angle_RS2]
    #         right_scan_avg = (laser_scan[self.angle_RS1] + laser_scan[self.angle_RS2])/2

    #         errors = [left_error, right_error]
    #         scan_avgs = [left_scan_avg, right_scan_avg]
    #         side_str = ["left", "right"]

    #         # Find the wall closet to robot
    #         min_error = float("inf")
    #         min_scan_avg = float("inf")
    #         min_error_side = None
    #         for i in range(len(errors)):
    #             if (not math.isnan(errors[i])) and (scan_avgs[i] < min_scan_avg):
    #                 min_error = errors[i]
    #                 min_error_side = side_str[i]
            
    #         # print("min_error: %s" % min_error)
    #         # print("side: %s" %min_error_side)

    #         if (min_error == float("inf") or abs(min_error) < 1e-2):
    #             self.velocity.angular.z = 0
    #         elif min_error < 0:
    #             self.velocity.angular.z = self.k_cw*min_error
    #         else:
    #             self.velocity.angular.z = self.k_cc*min_error



    #     # object detected infront of robot, start to follow
    #     else:
    #         print("somethiong in front")
    #         # points 90 degress to left and 90 degress to right
    #         # of the center of the neato
    #         self.compute_com_x(laser_scan)
    #         self.compute_com_y(laser_scan)
    #         self.adjust()


if __name__ == '__main__':
    node = FiniteStateControlNode()
    node.run()
