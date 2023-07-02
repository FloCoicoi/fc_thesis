#!/usr/bin/env python3

import rospy
from dwb_msgs.msg import LocalPlanEvaluation, TrajectoryScore, Trajectory2D
from geometry_msgs.msg import PoseArray, Pose, Quaternion
from tf.transformations import quaternion_from_euler


from numpy import pi
import numpy as np
import ros_numpy

"""
    This is a debugging script that helps visualizing the cotsmaps created
    by the trajectory critics in the dwb_local_planner.
"""


class TrajectoryVizualiser:
    def __init__(self):
        rospy.init_node("trajectory_visualizer", anonymous=True)
        self.array_publisher = rospy.Publisher("trajectory_end", PoseArray, queue_size=1)
        self.traj_subscriber = rospy.Subscriber(
            "/locomotor/DWBLocalPlanner/evaluation", LocalPlanEvaluation, self.trajCallback
        )
        self.full_traj = True  # Change manually to see full traj. Tip: choose small vx_samples and vtheta_samples

    def trajCallback(self, msg):
        output = PoseArray()
        output.header = msg.header
        # print(len(msg.twists)) #This helps
        for twist in msg.twists:
            if self.full_traj:
                for pose_2d in twist.traj.poses:
                    pose = Pose()
                    pose.position.x, pose.position.y = pose_2d.x, pose_2d.y
                    pose.orientation = Quaternion(*list(quaternion_from_euler(0.0, 0.0, pose_2d.theta)))
                    output.poses.append(pose)
            else:
                pose = Pose()
                pose.position.x, pose.position.y = twist.traj.poses[-1].x, twist.traj.poses[-1].y
                pose.orientation = Quaternion(*list(quaternion_from_euler(0.0, 0.0, twist.traj.poses[-1].theta)))
                output.poses.append(pose)
        self.array_publisher.publish(output)

    def listen(self):
        rospy.spin()


traj_viz = TrajectoryVizualiser()
traj_viz.listen()
