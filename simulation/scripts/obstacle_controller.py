#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

from numpy import pi
import numpy as np
import ros_numpy

"""
    This script commands a moving obstacle in gazebo. It should be started as a ROS node
    in the same launch file that spawns the gazebo object, and that loads a possible config
    file. A config file can be used to hold the rosparams used by this script:

    ######## Parameters:
    Name          Type     Default          Description
    obst_name     str      "obst1"          Name of the obstacle. Will subscribe to /obst_name/cmd_vel
    wait_topic    str      None             If specified, starts to move when something is received through that topic

"""


class ObstacleController:
    def __init__(self):

        # Handle rosparams
        self.obst_name = rospy.get_param("new_obst_name", "obst1")
        rospy.init_node("controller")  # This gets renamed by the launch file
        self.obst_name = rospy.get_name()
        self.wait_topic = rospy.get_param(self.obst_name + "/wait_topic", None)
        self.vx = rospy.get_param(self.obst_name + "/vx", 0.0)
        self.vy = rospy.get_param(self.obst_name + "/vy", 0.3)

        # Init node and subscribers/publishers
        self.cmd_pub = rospy.Publisher("cmd_vel_obst", Twist, queue_size=1)
        data = rospy.wait_for_message(self.wait_topic, rospy.AnyMsg, 1000)
        if data is None:
            rospy.loginfo("No Go received, Aborting")
        else:
            rospy.loginfo("Go!")
            self.start_controller()

    def shutdownCallback(self):
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        cmd_vel.linear.y = 0.0
        self.cmd_pub.publish(cmd_vel)

    def start_controller(self):

        # Start command loop
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            cmd_vel = Twist()
            cmd_vel.linear.x = self.vx
            cmd_vel.linear.y = self.vy
            self.cmd_pub.publish(cmd_vel)
            rate.sleep()


ObstacleController()
