#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

from numpy import pi
import numpy as np
import ros_numpy

"""
    This is a debugging script that helps visualizing the cotsmaps created
    by the trajectory critics in the dwb_local_planner.
"""


class CostmapVizualiser:
    def __init__(self):
        rospy.init_node("costmap_visualizer", anonymous=True)
        self.cloud_publisher = rospy.Publisher("filtered_costcloud", PointCloud2, queue_size=1)
        self.cloud_subscriber = rospy.Subscriber(
            "/locomotor/DWBLocalPlanner/cost_cloud", PointCloud2, self.cloudCallback
        )

    def cloudCallback(self, msg):

        # Read the point cloud data
        np_pc = ros_numpy.numpify(msg)
        # points = np_pc[np_pc["total_cost"] == np.max(np_pc["total_cost"])]
        # fields = [msg.fields[-1]]
        # head = msg.header
        # head.frame_id = "camera_left"
        # pc_msg = pc2.create_cloud(head, fields, points)
        field_to_watch = "total_cost"
        pc_cost = np_pc[field_to_watch]
        pc_x = np_pc["x"]
        pc_y = np_pc["y"]
        pc_z = np_pc["z"]
        filter = pc_cost < 0.3

        pc_cost, pc_x, pc_y, pc_z = pc_cost[filter], pc_x[filter], pc_y[filter], 10.0 * pc_cost[filter]

        points = []
        for k in range(len(pc_x)):
            points.append((pc_x[k], pc_y[k], pc_z[k], pc_cost[k]))
        head = msg.header
        names = ["x", "y", "z", field_to_watch]
        my_fields = []
        for k in range(len(msg.fields)):
            if msg.fields[k].name in names:
                my_fields.append(msg.fields[k])
        my_fields[-1].name = "score"
        pc_msg = pc2.create_cloud(head, my_fields, points)
        # rospy.loginfo(str(pc_msg.fields[0].count))

        # Filter the 2D line out of the 3D point cloud
        # points = []
        # for k in range(len(pc_x)):
        #     points.append((pc_x[k], pc_y[k], pc_z[k], 1))
        # head = msg.header
        # head.frame_id = "camera_left"
        # pc_msg = pc2.create_cloud(head, msg.fields, points)

        self.cloud_publisher.publish(pc_msg)

    def listen(self):
        rospy.spin()


visualizer = CostmapVizualiser()
visualizer.listen()
