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
    The algorithm considers ona angle at a time, and choses the closest depth
    among the points at that angle, between min_height and max_height (respective to the camera's height).
"""


class MinScan:
    def __init__(self):
        rospy.init_node("min_scan", anonymous=True)
        self.scan_publisher = rospy.Publisher("min_scan/scan", LaserScan, queue_size=1)
        self.cloud_subscriber = rospy.Subscriber("/pointcloud", PointCloud2, self.cloudCallback)
        self.pc_pub = rospy.Publisher("min_scan/pc", PointCloud2, queue_size=1)

    def filter_point_cloud(self, np_pc):
        # In this frame, x is the width, y is the height and z is the depth
        min_height = -0.3  # Respect to the camera's height
        max_height = 0.4

        pc_x = -np_pc["x"]
        pc_y = -np_pc["y"]
        pc_z = np_pc["z"]

        nan_filter = ~np.isnan(pc_x) & ~np.isnan(np_pc["y"]) & ~np.isnan(pc_z)
        pc_x, pc_y, pc_z = pc_x[nan_filter], pc_y[nan_filter], pc_z[nan_filter]

        x_filter = np.abs(pc_x) < (6.0 / 2)
        y_filter = (pc_y < max_height) & (pc_y > min_height)
        z_filter = pc_z > 0.05
        pc_filter = x_filter & y_filter & z_filter

        pc_x, pc_y, pc_z = pc_x[pc_filter], -pc_y[pc_filter], pc_z[pc_filter]
        return pc_x, pc_y, pc_z

    def cloudCallback(self, msg):

        start_time = rospy.Time.now().to_nsec()
        # Variables to make params from
        a_increment = pi * 0.001

        # Read the point cloud data
        np_pc = ros_numpy.numpify(msg)
        pc_x, pc_y, pc_z = self.filter_point_cloud(np_pc)

        # Filter the 2D line out of the 3D point cloud
        # points = []
        # for k in range(len(pc_x)):
        #     points.append((pc_x[k], pc_y[k], pc_z[k], 1))
        # head = msg.header
        # head.frame_id = "camera_left"
        # pc_msg = pc2.create_cloud(head, msg.fields, points)

        # Publish the "2D point cloud"
        # self.pc_pub.publish(pc_msg)

        # Process angles and depths
        angles = np.arctan(np.multiply(pc_x, 1 / pc_z))
        depths = np.sqrt(pc_x**2 + pc_z**2)

        # Init the LaserScan msg
        lscan = LaserScan()
        lscan.header.stamp = msg.header.stamp
        lscan.header.frame_id = "camera"
        lscan.angle_min = np.min(angles)
        lscan.angle_max = np.max(angles)
        lscan.angle_increment = a_increment
        lscan.range_min = np.min(depths)
        lscan.range_max = np.max(depths)
        lscan.ranges = [0.0] * int((lscan.angle_max - lscan.angle_min) / a_increment + 1)

        # Actually compute the ranges
        for idx in range(len(lscan.ranges)):
            # For each laserscan direction, keep the minimal depth among the points closer to that direction
            points_set = np.floor((angles - lscan.angle_min) / a_increment).astype(int) == idx
            if points_set.any():
                lscan.ranges[idx] = np.min(depths[points_set])
            else:
                lscan.ranges[idx] = lscan.range_max

        # Publish it
        self.scan_publisher.publish(lscan)
        rospy.loginfo("Processed in: " + str(rospy.Time.now().to_nsec() - start_time))

    def listen(self):
        rospy.spin()


min_scan = MinScan()
min_scan.listen()
