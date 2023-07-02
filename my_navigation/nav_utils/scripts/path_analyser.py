#!/usr/bin/env python3

import rospy
from geometry_msgs import PoseStamped, Odometry
from nav_msgs import Path

from numpy import pi
import numpy as np

class PathAnalyser():

    def __init__(self):
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odomCallback)
        self.goal_sub = rospy.Subscriber("goalstamped", PoseStamped, self.goalCallback)
        self.path_sub = rospy.Subscriber("/locomotor/DWBLocalPlanner/global_plan", Path, self.pathCallback)
        self.cmd_sub = rospy.Subscriber("cmd_vel", Twist, self.cmdCallback)

    def odomCallback(self, msg):
        # Convert the odom msg to a Pose2D kind of data
        new_pose = msg

        # Buffer it up!
        self.pose_buffer.append(new_pose)

    def goalCallback(self, msg):
        # Keep the goalstamped list in mind to criticize parts of the trajectory properly
        self.

    def pathCallback(self, msg):
        # Record the planned global path
        # path.poses is a list of PoseStamped

    def cmdCallback(self, msg):
        # Record cmd_vel
        cmd_list.append([cmd_vel.linear.x, cmd_vel.angular.z])

    def dist2path(self):
        """
        This ouput a simple distance to the global path. If there are several global paths it will always compare the
        robot's positions with the newest position available at the position's time.
        It measures the ability of the local planner to follow the global path. This measure can be used to compare
        two local planners in similar conditions (same environment, same global plan), and must be considered along
        with other metrics such as obstacle clearance.
        If the global plan is refreshed on schedule, the robot would be at the global path's beginning, so refreshing
        often the global path artificially reduces the distance to path output by this function.
        """

    def cmdVar(self):
        """
        This function returns the standard deviation of the cmd_vel derivative through the simulation.
        If a robot has large changes in the cmd_vel it means that it is oscillating in some way.
        """
        A = np.array(cmd_list)
        dA = np.gradient(A, axis=0)
        return np.std(dA, axis=0) # should output a list size 2 corresponding to the lin and angular variability
