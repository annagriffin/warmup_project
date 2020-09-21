
#!/usr/bin/env python3

import rospy
from sensor_mesgs.msg import LaserScan
from geometry_msgs import Twist, Vector3
from visualization_msgs import Marker

class SphereMarkerNode(object):
    def __init__(self):
        rospy.init_node('sphere_marker')

        self.marker = Marker()
        self.marker.header.frame_id = "base_link"
        self.marker.header.stamp = rospy.Time.now()
        self.marker.ns = "namespace"
        self.marker.id = 0
        self.marker.type = Marker.SPHERE
        self.marker.pose.position.x = 1
        self.marker.pose.position.y = 1
        self.marker.color.a = 1.0
        self.marker.color.r = 0.0
        self.marker.color.g = 1.0
        self.marker.color.b = 0.0
        
        self.publisher = rospy.Publish('/my_marker', Marker, queue_size=10)



    def process_marker(self, m):
        print(m)

    def run(self):
        # r = rospy.Rate(10)
        # while note rospy.is_shutdown():
        #     self.publisher(Marker)

if __name__ == '__main__':
    node = SphereMarkerNode()
    node.run()
