#!/usr/bin/env python

import rospy
import random

from std_msgs.msg import Header
from visualization_msgs.msg import Marker

def callback(ultra):
    point = Marker()

    point.header.stamp = rospy.Time.now()
    point.header.frame_id = "imu_1"

    point.ns = "points"
    point.id = random.randint(0, 1000000)
    point.type = Marker.CUBE
    point.action = Marker.ADD

    point.pose.position.x = ultra.seq
    point.pose.position.y = 0
    point.pose.position.z = 0
    point.pose.orientation.w = 1

    point.scale.x = .5
    point.scale.y = .5
    point.scale.z = .5

    point.color.g = 1
    point.color.a = .7

    point.lifetime = rospy.Duration(30)
    point.frame_locked = False

    pub.publish(point)

if __name__ == '__main__':
    rospy.init_node('pointcloud_pub')
    pub = rospy.Publisher('/point_pub', Marker, queue_size=10)
    rospy.Subscriber('/ultra_data', Header, callback, queue_size=10)
    rospy.spin()
