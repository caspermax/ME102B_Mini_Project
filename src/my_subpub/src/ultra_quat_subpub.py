#!/usr/bin/env python

import rospy
import tf2_ros

import madgwickahrs
from madgwickahrs import MadgwickAHRS
import quaternion
from quaternion import Quaternion
import numpy as np
from tf.transformations import quaternion_matrix

from std_msgs.msg import Header
from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker

from message_filters import ApproximateTimeSynchronizer, Subscriber

pub = rospy.Publisher('/pointcloud', Marker, queue_size=10)

def callback():
    print("hello")
    # R = Rotation.from_quat(quat_sub, ultra_sub)
    point = Marker()
    # point.pose.position.x = 
    # point.pose.position.y = 
    # point.pose.position.z = 
    pub.publish(point)

if __name__ == '__main__':
    quat_sub = Subscriber('/ts_stamped_pub', TransformStamped, queue_size=10)
    ultra_sub = Subscriber('/ultra_data', Header, queue_size=10)

    ats = ApproximateTimeSynchronizer([quat_sub, ultra_sub], queue_size=5, slop=0.5)
    ats.registerCallback(callback)