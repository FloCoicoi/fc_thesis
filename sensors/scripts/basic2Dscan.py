#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import LaserScan

from numpy import pi
import numpy as np
import ros_numpy

"""
    This script provides a lidar scan message based on the /pointcloud
    received information.
    The algorithm simply extracts the slice corresponding to the plane in front of
    the rover, at the camera's height, and transcribes these points to the LaserScan
    format, with ranges and angles. The ranges of the LaserScan is a mean processed through
    the points in the few slices just above and below the camera's height.
"""


class BasicScan:
    def __init__(self):
        rospy.init_node("basic_scan", anonymous=True)
        self.scan_publisher = rospy.Publisher("basic_scan/scan", LaserScan, queue_size=1)
        self.cloud_subscriber = rospy.Subscriber("/pointcloud", PointCloud2, self.cloudCallback)
        self.pc_pub = rospy.Publisher("basic_scan/pc", PointCloud2, queue_size=1)

    def filter_point_cloud(self, np_pc):
        # In this frame, x is the width, y is the height and z is the depth

        pc_x = np_pc["x"]
        pc_y = -np_pc["y"]
        pc_z = np_pc["z"]

        nan_filter = ~np.isnan(pc_x) & ~np.isnan(np_pc["y"]) & ~np.isnan(pc_z)
        pc_x, pc_y, pc_z = pc_x[nan_filter], pc_y[nan_filter], pc_z[nan_filter]

        x_filter = np.abs(pc_x) < (6.0 / 2)
        y_filter = (pc_y < 0.05) & (pc_y > -0.05)
        z_filter = pc_z > 0.05
        pc_filter = x_filter & y_filter & z_filter  # & y_filter

        pc_x, pc_y, pc_z = pc_x[pc_filter], -pc_y[pc_filter], pc_z[pc_filter]

        return pc_x, pc_y, pc_z

    def cloudCallback(self, msg):

        # Variables to make params from
        a_increment = pi * 0.001

        # Read the point cloud data
        np_pc = ros_numpy.numpify(msg)
        pc_x, pc_y, pc_z = self.filter_point_cloud(np_pc)

        # Filter the 2D line out of the 3D point cloud
        points = []
        for k in range(len(pc_x)):
            points.append((pc_x[k], pc_y[k], pc_z[k], 1))
        head = msg.header
        head.frame_id = "camera_left"
        pc_msg = pc2.create_cloud(head, msg.fields, points)

        # Publish the "2D point cloud"
        self.pc_pub.publish(pc_msg)

        # Process into a laserscan
        angles = np.arctan(np.multiply(pc_x, 1 / pc_z))
        rospy.loginfo((np.size(angles)))
        depths = np.sqrt(pc_x**2 + pc_z**2)

        # Compute the LaserScan msg
        lscan = LaserScan()
        lscan.header.stamp = msg.header.stamp
        lscan.header.frame_id = msg.header.frame_id
        lscan.angle_min = np.min(angles)
        lscan.angle_max = np.max(angles)
        lscan.angle_increment = a_increment
        lscan.range_min = np.min(depths)
        lscan.range_max = np.max(depths)
        ranges = [0.0] * int((lscan.angle_max - lscan.angle_min) / a_increment + 1)
        counts = [0] * int((lscan.angle_max - lscan.angle_min) / a_increment + 1)

        for k in range(np.size(angles)):
            idx = int((angles[k] - lscan.angle_min) / a_increment)
            ranges[idx] += depths[k]
            counts[idx] += 1
        for idx in range(len(ranges)):
            if counts[idx] == 0:
                ranges[idx] = lscan.range_max
            else:
                ranges[idx] *= 1 / counts[idx]

        lscan.ranges = ranges

        # Publish it
        self.scan_publisher.publish(lscan)

    def listen(self):
        rospy.spin()


basic_scan = BasicScan()
basic_scan.listen()
