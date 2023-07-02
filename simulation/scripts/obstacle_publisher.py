#!/usr/bin/env python3

import rospy
from obstacle_detector.msg import Obstacles, CircleObstacle
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

from numpy import pi
import numpy as np

"""
    This script works hand in hand with the teleoperated obstacles. Its assumptions are that the obstacles have
    their odometry message and odometry frame named "<obstacle_name>_odom"
    The only parameter for this script is the list of obstacles names. The node then subscribes to the corresponding
    odometry topics and publishes obstacles message at 100Hz and with the pcl_filtered freshest stamps.
"""


class ObstaclePublisher:
    def __init__(self):

        # Handle rosparams
        rospy.init_node("obst_pub")
        self.obst_names = rospy.get_param("~obst_names").split(", ")
        self.vel_noise_std = rospy.get_param("~vel_noise_std", 0.0)
        self.pose_noise_std = rospy.get_param("~pose_noise_std", 0.0)
        self.pub_topic = rospy.get_param("~pub_topic", "obstacles")

        k = 0
        self.name_ids = {}
        for name in self.obst_names:
            self.name_ids[name] = k
            k += 1

        # Init the obst publish params
        self.odom_subs = []
        self.obst_msg = Obstacles()
        self.obst_msg.header.frame_id = "map"
        self.obst_pub = rospy.Publisher(self.pub_topic, Obstacles, queue_size=1)

        # For each obstacle name
        for name in self.obst_names:
            # Init publisher msg
            new_circle = CircleObstacle()
            new_circle.radius = 0.6
            new_circle.true_radius = 0.5
            self.obst_msg.circles.append(new_circle)

            # Init subscriber
            sub = rospy.Subscriber(name + "_odom", Odometry, self.odomCallback)
            self.odom_subs.append(sub)

        # Subscribe to the pcl_filter just to get the time stamp
        self.pcl_sub = rospy.Subscriber("pcl_filtered", PointCloud2, self.stampCallback)
        self.timer = rospy.Timer(rospy.Duration(0.04), self.publish_obst)
        rospy.spin()

    def odomCallback(self, msg):
        # Find the obstacle name (obstname_odom)
        n = self.name_ids[msg.header.frame_id[:-5]]
        self.obst_msg.circles[n].center.x = msg.pose.pose.position.x + np.random.normal(0.0, self.pose_noise_std)
        self.obst_msg.circles[n].center.y = msg.pose.pose.position.y + np.random.normal(0.0, self.pose_noise_std)
        self.obst_msg.circles[n].velocity.x = msg.twist.twist.linear.x * (
            1 + np.random.normal(0.0, self.vel_noise_std)
        )
        self.obst_msg.circles[n].velocity.y = msg.twist.twist.linear.y * (
            1 + np.random.normal(0.0, self.vel_noise_std)
        )

    def stampCallback(self, msg):
        self.obst_msg.header.stamp = msg.header.stamp

    def publish_obst(self, time):
        self.obst_pub.publish(self.obst_msg)


ObstaclePublisher()
