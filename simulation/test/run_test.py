#!/usr/bin/env python3

import roslaunch
import rosnode
import rospy
from geometry_msgs.msg import PoseStamped, Twist, TransformStamped
from obstacle_detector.msg import Obstacles
import tf2_ros
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import DeleteModel, DeleteModelRequest
import message_filters

import math
import numpy as np
from datetime import datetime

################################
## User variables HERE! Make sure to check all values before running a test.
# You can always edit the result file to add details
test_move_base = False
robot_radius_ = 0.3  # used in collision check as the obstacle information does not include it
planner_name = "AdaptedVO (noise)"
# obst_names = []
# obst_x = []
# obst_y = []
# obst_vx = []
# obst_vy = []
obst_names = ["perp", "front", "dont_overtake", "overtake"]
obst_x = [4.3, 8.0, 1.0, 2.0]
obst_y = [-4.4, 0.0, -2.0, -0.1]
obst_vx = [0.0, -0.7, 0.3, 0.35]
obst_vy = [0.55, 0.0, 0.45, 0.0]
pose_std = 0.001  # Almost none in practical cases
vel_std = 0.001  # Quite high in practice. A value of 0.2 is crazy high
nb_rng_scenario = 10
rng_seed = 3

################################
# Scenario generation here!
rng = np.random.default_rng(seed=rng_seed)  # Fixed seed for comparability and reproducibility
nb_gen = 0
while nb_gen < nb_rng_scenario:
    # Generate interesting values for x, y, vx, vy:
    coll_point = rng.random() * 4.0 + 2.0  # Collision within [2.0:6.0] meters in front of the robot
    x = rng.random() * 8.0  # From 0.0 to 9.0
    y = rng.random() * 10.0 - 5.0  # From -5.0 to 5.0
    while x**2 + y**2 < 2.0:  # If it is not in legal bounds, re-roll
        x = rng.random() * 9.0
        y = rng.random() * 10.0 - 5.0

    # Then compute velocity to get to collision_point at the same time as the robot from the x, y position
    t = coll_point / 0.7 + 1.0  # Time for the robot to get to the collision point (+1.0 due to some delay)
    t = t + 0.5 * rng.random()  # Plus some noise to spice it up
    vx = -(x - coll_point) / t
    vy = -y / t
    nb_gen += 1
    obst_names.append("rnd_obst_" + str(nb_gen))
    obst_x.append(x)
    obst_y.append(y)
    obst_vx.append(vx)
    obst_vy.append(vy)
# end of scenario generation

################################
# Init measurements
starttime = datetime.now()
successes = []
times = []
poses = []  # poses contains lists of measurements for each scenar. One measurement is pose (tuple)
obst_poses = []  # contains lists of measurements for each scenar. One measurement is a list obstacles (tuples)
all_dists = []
all_times = []
all_curvatures = []
all_obst_dists = []
meas_poses = False

################################
# def callbacks
def odomObstCallback(odommsg, obstmsg):
    global times, obst_poses, poses, meas_poses
    if not meas_poses:
        return

    # Obstacle info
    obst_info = []
    for circ in obstmsg.circles:
        obst_info.append((circ.center.x, circ.center.y, circ.true_radius))
    obst_poses[-1].append(obst_info.copy())

    # Odometry
    roll, pitch, yaw = euler_from_quaternion(
        (
            odommsg.pose.pose.orientation.x,
            odommsg.pose.pose.orientation.y,
            odommsg.pose.pose.orientation.z,
            odommsg.pose.pose.orientation.w,
        )
    )
    poses[-1].append((odommsg.pose.pose.position.x, odommsg.pose.pose.position.y, yaw))
    times[-1].append(odommsg.header.stamp.to_sec())


################################
## Init the magic
rospy.set_param("/use_sim_time", True)
rospy.init_node("test_manager", anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

## Launch the simulation
launch_args1 = [
    "simulation",
    "simulation.launch",
    "world_name:=large_room_bis",
]
sim_launch_name = roslaunch.rlutil.resolve_launch_arguments(launch_args1)[0]
sim_launch_args = launch_args1[2:]
sim_launch = roslaunch.parent.ROSLaunchParent(uuid, [(sim_launch_name, sim_launch_args)])
sim_launch.start(auto_terminate=False)  # auto_terminate=False allows to launch several files in a row
rospy.sleep(3)  # Wait for gazebo to start

## Start a static tf broadcaster, usually in the scenario launch files
broadcaster = tf2_ros.StaticTransformBroadcaster()
tf_mapodom = TransformStamped()
tf_mapodom.child_frame_id = "odom"
tf_mapodom.header.frame_id = "map"
tf_mapodom.header.stamp = rospy.Time.now()
tf_mapodom.transform.rotation.w = 1.0
broadcaster.sendTransform(tf_mapodom)

################################
## Prepare other launch file names
obst_launch_name = roslaunch.rlutil.resolve_launch_arguments(["simulation", "linear_waiter.launch"])[0]

if test_move_base:
    # move_base setup
    nav_launch_name = roslaunch.rlutil.resolve_launch_arguments(["simulation", "move_base.launch"])[0]
    nav_launch_args = [
        "map_name:=large_room_bis",
        "record_rosbag:=true",
        "use_amcl:=false",
    ]
else:
    # robot_navigation setup
    nav_launch_name = roslaunch.rlutil.resolve_launch_arguments(["simulation", "robot_nav.launch"])[0]
    nav_launch_args = [
        "map_name:=large_room_bis",
        "obstacle_detection:=false",
        "record_rosbag:=true",
        "use_amcl:=false",
        "use_pcl:= false",
    ]
# end of launch file setup

################################
## Gazebo rover teleporter and obst destroyer
gazebo_teleporter = rospy.Publisher("/gazebo/set_model_state", ModelState, queue_size=2)
rover_model_zero = ModelState()
rover_model_zero.model_name = "4WD_rover"
rover_model_zero.pose.position.z = 0.4  # slight drop from above the floor to make sure it won't clip through
obst_destroyer = rospy.ServiceProxy("/gazebo/delete_model", DeleteModel)

## Init the goal publisher
if test_move_base:
    goal_topic = "move_base_simple/goal"
else:
    goal_topic = "goalstamped"
goal_pub = rospy.Publisher(goal_topic, PoseStamped, queue_size=1)
goal = PoseStamped()
goal.pose.position.x = 6.5
goal.pose.orientation.w = 1.0
goal.header.frame_id = "map"

## Init the /odom subscriber
odom_sub = message_filters.Subscriber("/odom", Odometry)
obst_sub = message_filters.Subscriber("/perfect_obstacles", Obstacles)
ts = message_filters.ApproximateTimeSynchronizer([odom_sub, obst_sub], 10, 0.1, allow_headerless=True)
ts.registerCallback(odomObstCallback)
# odom_sub = rospy.Subscriber("/odom", Odometry, odomCallback, queue_size=1)
cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
zero_twist = Twist()


#######################################################################################
## Start running the tests
for obst_id in range(len(obst_x)):
    ## Init measurements
    poses.append([(0, 0, 0)])
    times.append([0.0])
    obst_poses.append([[(obst_x[obst_id], obst_y[obst_id], 0.6)]])

    ## Setup the obstacle launch file and launch everything
    obst_launch_args = [
        "obst_name:=" + str(obst_names[obst_id]),
        "x:=" + str(obst_x[obst_id]),
        "y:=" + str(obst_y[obst_id]),
        "vx:=" + str(obst_vx[obst_id]),
        "vy:=" + str(obst_vy[obst_id]),
        "publish_obst:=true",
        "run_publisher:=true",
        "pose_noise_std:=" + str(pose_std),
        "vel_noise_std:=" + str(vel_std),
    ]
    obst_launch = roslaunch.parent.ROSLaunchParent(uuid, [(obst_launch_name, obst_launch_args)])
    nav_launch = roslaunch.parent.ROSLaunchParent(uuid, [(nav_launch_name, nav_launch_args)])
    nav_launch.start(auto_terminate=False)
    obst_launch.start(auto_terminate=False)
    rospy.sleep(1)  # Wait for nodes to start

    ## Wait for dwb_local_planner to subscribe to goalstamped and wait one extra second to be sure it has started
    good = False
    if test_move_base:
        while not good:
            infos = rosnode.get_node_info_description("/move_base")
            if "move_base_simple/goal" in infos:
                good = True
    else:
        while not good:
            infos = rosnode.get_node_info_description("/locomotor")
            if "LocalPlanner/goal" in infos:
                good = True
    rospy.sleep(2)

    # Now start recording stuff
    meas_poses = True

    ## Publish the goalstamped then wait for the robot to reach the goal (max 20s to do so)
    goal.header.stamp = rospy.Time.now()
    goal_pub.publish(goal)
    good = False
    t = 0
    while not (good or t > 200):
        # If closer than 0.25 meters from the goal it's ok
        pose = poses[-1][-1]
        if (pose[0] - goal.pose.position.x) ** 2 + (pose[1] - goal.pose.position.y) ** 2 < 0.0625:
            good = True
        for obpose in obst_poses[-1][-1]:
            if (pose[0] - obpose[0]) ** 2 + (pose[1] - obpose[1]) ** 2 < (robot_radius_ + obpose[2]) * (
                robot_radius_ + obpose[2]
            ):
                # There is a collision
                t = 200
        # Wait .1 sec
        rospy.sleep(0.1)
        t += 1
    if good:
        # Then this test reached the goal, obstacle collisions are checked later
        successes.append(1)

    else:
        successes.append(0)
    meas_poses = False

    #################################################################################################
    # Pre-process and shutdown after test

    ## Shutdown the nav and obst launch files
    obst_launch.shutdown()
    nav_launch.shutdown()
    rospy.sleep(1)

    ## Publish zero cmd_vel to stop the rover and teleport it back at the start
    cmd_pub.publish(zero_twist)
    gazebo_teleporter.publish(rover_model_zero)

    ## Teleport the obstacle out of bounds, because I can't make DeleteModel work
    obst_model_state = ModelState()
    obst_model_state.model_name = obst_names[obst_id]
    obst_model_state.pose.position.z = 2.0
    obst_model_state.pose.position.x = -3
    gazebo_teleporter.publish(obst_model_state)

    ## Delete obstacle
    # req = DeleteModelRequest()
    # req.model_name = str(obst_names[obst_id])
    # resp = obst_destroyer(str(obst_names[obst_id])) # resp = obst_destroyer(req)
    # if not resp:
    #     rospy.loginfo("Obstacle '" + str(obst_names[obst_id]) + "' could not be deleted")
    # # Note: "rosservice call /gazebo/delete_service Computer_01_001" does delete the computer in bookstore
    # # But: the same with obst1 in any launchable scenario crashes gazebo

    ## Process the results of this run
    curvature = 0.0
    tot_dist = 0.0
    obst_dist_mean = 0.0
    obst_dist_scale = 0
    prev_pose = (0.0, 0.0, 0.0)
    for k in range(len(poses[-1])):
        pose = poses[-1][k]
        for obpose in obst_poses[-1][k]:
            obst_dist_scale += 1
            dist = math.sqrt((obpose[0] - pose[0]) ** 2 + (obpose[1] - pose[1]) ** 2)
            obst_dist_mean += dist
            if dist < obpose[2] + robot_radius_:
                successes[-1] = 0  # In case the robot hit the obstacle but got to goal after, this must be updated
        tot_dist += math.sqrt(
            (pose[0] - prev_pose[0]) * (pose[0] - prev_pose[0]) + (pose[1] - prev_pose[1]) * (pose[1] - prev_pose[1])
        )
        curvature += abs(pose[2] - prev_pose[2])
        prev_pose = pose
    obst_dist_mean = obst_dist_mean / obst_dist_scale
    all_obst_dists.append(obst_dist_mean)
    all_dists.append(tot_dist)
    all_curvatures.append(curvature)
    all_times.append(times[-1][-1] - times[-1][1])

    ## End of the for loop: next test will launch
    rospy.sleep(0.1)
# end of running tests

## Prepare writing into result file
res_file = open("test_results.txt", "a")
res_file.write("\n \n")
res_file.write(starttime.strftime("%b-%d-%Y %H:%M:%S"))
res_file.write("  Using: " + planner_name + " (rng_seed = " + str(rng_seed) + ")")

## Output stuff right now
rospy.loginfo("The END! \n")
print("Scenario   Success  Time    Path length  Curvature   ObstClearance")
res_file.write("\n Scenario      Success  Time     Path length   Curvature   ObstClearance")
for k in range(len(obst_names)):
    output = (
        "\n"
        + f" {obst_names[k]} "
        + (16 - len(obst_names[k])) * " "
        + f"{successes[k]}     {all_times[k]:.3f}     {all_dists[k]:.3f}         {all_curvatures[k]:.3f}         {all_obst_dists[k]:.3f} "
        + "\n"
    )
    print(output)
    res_file.write(output)

res_file.close()
