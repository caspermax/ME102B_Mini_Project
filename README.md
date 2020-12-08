# ME102B_Mini_Project
A "handheld scanner". Takes sensor data and broadcasts it for RVIZ.

ROS nodes are in src/my_subpub/src. The Arduino node that publishes the IMU and ultrasonic data is in src/my_subpub/src/mpu_pub
- mpu_pub.ino: A ROS node that runs on the Mega 2560 and publishes IMU and ultrasonic data to their respective topics
- quat_subpub.py: Reads IMU data off respective topic and uses a Madgwick filter to produce and publish quaternions that RVIZ can view.
- ultra_subpub.py: Reads ultrasonic data off respective topic and publishes distances with respect to the frame of the scanner that RVIZ can view. The effect is a 3D point map.
