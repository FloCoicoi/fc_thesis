# Parameter Guide for the robot_navigation Package

The robot_navigation package is an improved version of the ros navigation package. For this former version, a "Getting started" guide is available and provides tutorials for publishing an odometry, managing the transforms, etc... These steps are of course also required for the robot_navigation package, and can be achieved regardless of the navigation package that is used.

On the other hand, the navigation package also had good documentation to getting started with writing all the parameters needed by the planners in yaml files, and loading them using a template launch file. But there is a cruel lack for such help for the robot_navigation package.

The only example file I could find was [here](https://answers.ros.org/question/357516/getting-started-with-the-robot_navigation-stack/), where someone asked for help with the robot_navigation parameters, and answered to his own question a few months later with a quite extensive parameter file.

I had to dig in the dwb_local_planner code to make sure of how the parameters are used, and to find some new ones.

## Launch File Template

The robot_nav.launch file shows an example of a launch file used to start everything required for the autonomous navigation. A few things can be noted there:

 * The robot_navigation package does not provide an AMCL, so I recommend using that of the navigation package
 * The locomotor node used by the robot_navigation package does not listen to the same simple_goal topic as the navigation stack did, and does not even use the same msg type. To help with this matter, the locogoal_pub node makes the conversion from a PoseStamped goal to a 2DPose goal.
 * It is also possible to change the topic published by rviz through the publish goal button. The rviz_config file has this feature already. To modify another rviz config file, simply Ctrl+F "rviz/SetGoal" and modify the goal topic below.

The actual launch of the robot_navigation main node uses several yaml files. Please find below some help regarding these parameters.

## Costmap Parameters

The costmaps work the same as in the robot_navigation file. You can use the example files I provide, and find help with the regular costmap documentation to learn more about these parameters.

## Global Planner Parameters

I did not experiment much with the global planners as the documentation provided in the [robot_navigation/dlux_plugins/README.md](https://github.com/locusrobotics/robot_navigation/tree/noetic/dlux_plugins) gives examples of results provided by several global planners.

In your DluxGlobal Planner parameters, you might want to change the following:

 * neutral_cost: is the cost of crossing a cell. It is added to the value of this cell (high cell values are for occupied cells). The smaller this value, the more the planner will consider following longer paths if it avoids high cell values.
 * unknown_interpretation (free/expensive/lethal) tells how the unknown cells should be perceived by the global planner.
 * path_caching: true allows for regular re-planning attempts, and the improvement_threshold parameters tells how much a new global path should beat the previous one for the global plan to be updated.
 * Available potential_calculator and traceback classes are listed in the [dlux_plugins documentation](https://github.com/locusrobotics/robot_navigation/tree/noetic/dlux_plugins).

## Local Planner Parameters

The local planner's features are done by plugins. Thus every feature is easily modified. Keep in mind the way the local planner works: it calls a trajectory generator, that forward simulates trajectories (i.e. list of positions, with their timestamps) for several cmd_vel. These cmd_vel are selected by the xy_theta_iterator, but it can be seen as a part of the trajectory_generator. The generated trajectories are then scored by plugins called critics, and the lower the score the better the trajectory and thus the better the associated cmd_vel.

### DWB General parameters

* prune_plan: true and prune_plan_distance. The local planner considers a portion of the global plan to help score the possible trajectories. The global path that is behind a prun_plan_distance of the robot (or rather the projection of the robot's pose on the global path) will be erased.
* short_circuit_trajectory_evaluation: true simply makes it so that when scoring a trajectory, if the score given by the first few critics already is worse than the current best trajectory, there is no need to compute the score given by the next critic s in the list as it will only get even worse. This saves time with no counter parts. One might want to set it to false to ease the visualization and debugging of some critics.
* publish_cost_grid_pc and publish_evaluation are debugging tools. Some critics use some sort of costmap, and if you want to visualize it you need it specified. Same for the evaluation topic, which lists all the trajectories and the scores given by each critic. To help visualize these data, see my nav_utils package.

### Goal checker

The goal checker is frequently called to check if the goal is reached. There is not much about it, but you can use the simple GoalChecker if you don't care about your robot stopping when it reaches the goal, and call the StoppedGoalChecker if you want it stopped. Then you can set:
* rot_stopped_velocity and trans_stopped_velocity to define the maximum velocity the robot can have for the goal to be considered reached.
* xy_goal_tolerance and yaw_goal_tolerance tell how close to the goal the robot must be.

### Trajectory Generator

You can simply set your trajectory generator name if you wish to change it.

* The min_vel, max_vel, min_speed and max_speed parameters define the robot's limit velocities. They are quite straight forward and resemble the navigation parameters.
* The acc_lim and decel_lim parameters should match your robot's real limits for the trajectory generator to provide good trajectory estimate, and to make sure the planner does not plan on velocities your robot could not reach due to its actual acceleration limits.
* v[]_samples tell how many different velocities to try when generating trajectories. Having too few gives a very sparse exploration of the reachable space, but having many increases the computation time.
* sim_time is the time to forward simulate your trajectories. Of course planning will be more computationally demanding if you generate longer trajectories. Having a long sim_time can be bad if you have wrong kinematic parameters. Having it longer also requires more velocity samples for finer movements, or all of the robots trajectories will end up far from its current position, and the cloud of your trajectory endpoints will be very sparse.
* linear_granularity and angular_granularity tell how to discretize the trajectories generated. I did not thoroughly looked into the trajectory generator for now so I am a bit unsure how these parameters are used.

### Critics

Each critic gives a new criterion to define what a good trajectory is. But combining many criteria can be very difficult.

* default_critic_namespaces: This list of namespaces tells the DWB planner where to look for critics. If you make your own in a different package and with a different namespace, you must add it to the list. Make sure you have the fc_thesis/third_party/dwb_local_planner in your catkin workspace. It contains a fix to the dwb_planner that made wrong use of this parameter.
* critics: is the list of critics the DWB planner will attempt to laod. A critic class must end with "Critic", but you can put it in this list with or without the "Critic" at the end. The loader will try the namespaces in the given order when attempting to load the critics. When scoring a trajectory, the critics are called in the given order. This has some importance because if you set the short_circuit_trajectory to true (which you should), the scoring of a trajectory will stop when the score gets higher than the already highest scored trajectory. Thus you should put in front the critics that are fast to compute and that are the most important (with high score values).

Then you can specify parameter to each critic. To know what parameters each critic can take, you need to look for them in the code. They all have a "scale" parameter which weights the score given by the critic when adding it to the total count. Beware that the grid based critics have an invisible multiplication by map_resolution*0.5 to which the scale multiplies.

To balance the local planner's decision making you must know what kind of values each critic outputs, so that you can choose the weights wisely.
