#!/usr/bin/env python

import rospy
import tf2_ros

import madgwickahrs
from madgwickahrs import MadgwickAHRS
import quaternion
from quaternion import Quaternion
import numpy as np

from sensor_msgs.msg import Imu
from geometry_msgs.msg import QuaternionStamped
from geometry_msgs.msg import TransformStamped

mad = MadgwickAHRS(sampleperiod=.001, # 1000 DPS
                   quaternion=Quaternion(1, 0, 0, 0),
                   beta=1)

""" Save data to CSV """
def save_data(gyro, accel, mag):
    with open('gyro_data.csv', 'a+') as gyro_file:
        gyro_csv_str = ','.join(['%.5f' % num for num in gyro])
        gyro_file.write(gyro_csv_str + '\n')

    with open('accel_data.csv', 'a+') as accel_file:
        accel_csv_str = ','.join(['%.5f' % num for num in accel])
        accel_file.write(accel_csv_str + '\n')

    with open('mag_data.csv', 'a+') as mag_file:
        mag_csv_str = ','.join(['%.5f' % num for num in mag])
        mag_file.write(mag_csv_str + '\n')

def imu_pose(msg, imu_name):
    br = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    
    """ Read data from Arduino+IMU publisher node """
    degtorad = np.pi/180
    gyro = [msg.orientation.x * degtorad - 4.231, 
            msg.orientation.y * degtorad - 0.438, 
            msg.orientation.z * degtorad + 0.0724]
    accel = [msg.angular_velocity.x, 
             msg.angular_velocity.y, 
             msg.angular_velocity.z]
    mag = np.array([msg.linear_acceleration.x, 
                    msg.linear_acceleration.y, 
                    msg.linear_acceleration.z])
    
    """ Adjust for magnetic offset. Values from MotionCal """
    mag_soft_arr = np.array([[0.988, 0.008, 0.056],
                             [0.008, 1.025, 0.031],
                             [0.056, 0.031, 0.991]])
    mag_hard_off = np.array([11.85, 8.91, 11.65])
    mag = np.matmul(mag_soft_arr, mag)
    mag = mag - mag_hard_off
    mag = np.ndarray.tolist(mag)

    """ Save data to CSV """
    save_data(gyro, accel, mag)

    """ Calculate Quaternion using Madgwick Filter """
    mad.update(gyro, accel, mag)
    q = mad.quaternion._get_q()

    """ Write and broadcast TF transform """
    t.header.seq = msg.header.seq
    t.header.stamp = msg.header.stamp
    t.header.frame_id = "world"
    t.child_frame_id = imu_name

    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0

    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)
    pub.publish(t)


if __name__ == '__main__':
    rospy.init_node('tf2_imu_broadcaster')
    imu_name = 'imu_1'
    pub = rospy.Publisher('/ts_stamped_pub', TransformStamped, queue_size=10)
    rospy.Subscriber('/imu_data', Imu, imu_pose, imu_name, queue_size=10)
    rospy.spin()

