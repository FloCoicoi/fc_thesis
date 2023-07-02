#!/usr/bin/env python3

import rospy
from dwb_msgs.msg import LocalPlanEvaluation, TrajectoryScore, Trajectory2D
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import sys


from numpy import pi
import numpy as np
import ros_numpy

"""
    This is a debugging script that helps visualizing the cotsmaps created
    by the trajectory critics in the dwb_local_planner.
"""


class ScoreTrajectoryVizualiser:
    def __init__(self, crit_name):
        rospy.init_node("scoretrajectory_visualizer", anonymous=True)
        self.cloud_publisher = rospy.Publisher("trajectory_pc", PointCloud2, queue_size=1)
        self.traj_subscriber = rospy.Subscriber(
            "/locomotor/DWBLocalPlanner/evaluation", LocalPlanEvaluation, self.trajCallback
        )
        self.critic_name = crit_name

    def trajCallback(self, msg):
        """
        msg.twists is a list of TrajectoryScores
        Then each twist in msg.twists has a traj and a scores:
            *traj has velocity and poses (list of 2Dpose)
            *scores is a list of CriticScore, with keys name, raw_score and scale
        """
        # If we want to see every critic
        if self.critic_name == "":
            critic_dist = {}  # Stores encountered critics
            critic_max = {}
        points = []

        # Store data
        for twist in msg.twists:
            if twist.total > 0:
                # Get score info
                score_val = []
                for critic in twist.scores:
                    # If we want to see every critic
                    if self.critic_name == "":
                        # If critic never seen, store it
                        if not (critic.name in critic_dist):
                            critic_dist[critic.name] = len(critic_dist)
                            critic_max[critic.name] = critic.raw_score
                        score_val.append(critic.raw_score)
                        if critic_max[critic.name] < critic.raw_score:
                            critic_max[critic.name] = critic.raw_score
                    # else just keep the one we want
                    elif self.critic_name == critic.name:
                        score_val.append(critic.raw_score)
                        # rospy.loginfo(str(critic.raw_score * critic.scale))
                # If we want a specific critic, this ensures we have it
                if len(score_val) > 0:
                    for pose in twist.traj.poses:
                        points.append([pose.x, pose.y, 0] + score_val + [twist.total])

        # Set up the pc fields
        if self.critic_name == "":
            info_names = ["x", "y", "z"] + list(critic_dist.keys()) + ["total"]
        else:
            info_names = ["x", "y", "z"] + [self.critic_name, "total"]
        my_fields = []
        for count, name in enumerate(info_names):
            field = PointField()
            field.name, field.offset, field.datatype, field.count = name, 4 * count, 7, 1
            my_fields.append(field)

        # If we want to see every critic, some adjustment is needed
        if self.critic_name == "":
            for n in range(len(points)):
                if len(points[n]) < len(my_fields):
                    tot = points[n][-1]
                    points[n][-1] = critic_max[my_fields[len(points[n]) - 1].name]
                    for k in range(len(points[n]), len(my_fields) - 1):
                        points[n].append(critic_max[my_fields[k].name])
                    points[n].append(tot)
        # Write the output msg
        head = msg.header
        pc_msg = pc2.create_cloud(head, my_fields, points)
        self.cloud_publisher.publish(pc_msg)

    def listen(self):
        rospy.spin()


crit_name = ""
if len(sys.argv) > 2:
    rospy.loginfo("Two many arguments given. Ignored")
if len(sys.argv) == 2:
    crit_name = sys.argv[1]
traj_viz = ScoreTrajectoryVizualiser(crit_name)
traj_viz.listen()
