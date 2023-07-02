#!/usr/bin/env python3
from matplotlib.pyplot import axis
import numpy as np
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image 
import os
import time
import tf2_ros
from geometry_msgs.msg import TransformStamped
from tf.transformations import quaternion_from_euler
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from razor_imu_9dof.msg import orientation


class odometry_information():
    def __init__(self): 
        rospy.init_node("imu_data") 
        # self.imu_sub = rospy.Subscriber("/imu", Imu, self.imu_callback)
        self.orient_sub = rospy.Subscriber("/orientation",orientation, self.orient_callback)
        self.odom_pubs = rospy.Publisher("Odometry", Odometry, queue_size = 10)
        self.odometry = Odometry()
        self.q_x = 0.0
        self.q_y = 0.0
        self.q_z = 0.0
        self.q_w = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0
    def orient_callback(self, orient):
        self.roll = orient.roll
        self.pitch = orient.pitch
        self.yaw = orient.yaw


        self.odometry.pose.pose.position.x = 0.0
        self.odometry.pose.pose.position.y = 0.0
        self.odometry.pose.pose.position.z = 0.0

        self.q_x, self.q_y, self.q_z, self.q_w = quaternion_from_euler(
                                                0., 
                                                0.,
                                                self.yaw
                                                )
        self.odometry.pose.pose.orientation.x = self.q_x
        self.odometry.pose.pose.orientation.y = self.q_y
        self.odometry.pose.pose.orientation.z = self.q_z
        self.odometry.pose.pose.orientation.w = self.q_w
        self.odometry.header.frame_id = "imu"
        self.odometry.child_frame_id = "base_imu_link"
        self.odometry.header.stamp = rospy.Time.now()
        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.odometry.header.frame_id
        t.child_frame_id = self.odometry.child_frame_id
        t.transform.translation.x = self.odometry.pose.pose.position.x
        t.transform.translation.y = self.odometry.pose.pose.position.y
        t.transform.translation.z = self.odometry.pose.pose.position.z
        t.transform.rotation.x = self.odometry.pose.pose.orientation.x
        t.transform.rotation.y = self.odometry.pose.pose.orientation.y
        t.transform.rotation.z = self.odometry.pose.pose.orientation.z
        t.transform.rotation.w = self.odometry.pose.pose.orientation.w
        br.sendTransform(t)
        print("imu_success", self.q_z)
        self.odom_pubs.publish(self.odometry)
    """    
    def imu_callback(self, imu_msgs):
        self.q_x = imu_msgs.orientation.x
        self.q_y = imu_msgs.orientation.y
        self.q_z = imu_msgs.orientation.z
        self.q_w = imu_msgs.orientation.w

        self.odometry.pose.pose.position.x = 0.0
        self.odometry.pose.pose.position.y = 0.0
        self.odometry.pose.pose.position.z = 0.0
        self.odometry.pose.pose.orientation.x = self.q_x
        self.odometry.pose.pose.orientation.y = self.q_y
        self.odometry.pose.pose.orientation.z = self.q_z
        self.odometry.pose.pose.orientation.w = self.q_w
        self.odometry.header.frame_id = "imu"
        self.odometry.child_frame_id = "base_imu_link"
        self.odometry.header.stamp = rospy.Time.now()

        br = tf2_ros.TransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = self.odometry.header.frame_id
        t.child_frame_id = self.odometry.child_frame_id
        t.transform.translation.x = self.odometry.pose.pose.position.x
        t.transform.translation.y = self.odometry.pose.pose.position.y
        t.transform.translation.z = self.odometry.pose.pose.position.z
        t.transform.rotation.x = self.odometry.pose.pose.orientation.x
        t.transform.rotation.y = self.odometry.pose.pose.orientation.y
        t.transform.rotation.z = self.odometry.pose.pose.orientation.z
        t.transform.rotation.w = self.odometry.pose.pose.orientation.w
        br.sendTransform(t)
        print("imu_success", self.q_z)
        self.odom_pubs.publish(self.odometry)
    """
if __name__ == "__main__":
    odoms = odometry_information()
    rospy.spin()
    
