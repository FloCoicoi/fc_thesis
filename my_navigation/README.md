# README Navigation

This folder goes in addition with the installed ROS navigation stack.

## Collision_avoidance

This package can provide emergency behaviors for the robot. For now it contains a node able to give rumble feedback in the ps4 controller in case a obstacle is detected close to the robot.

## Goal_publisher

The goal_publisher will contain more functionalities such as a moving goal publisher, or a teleoperable goal to simulate a follow-me scenario. For now it subscribe to the goal topic on which the user can publish Pose messages, and it makes the lacking data to make a PoseStamped message and publish it on the move_base goal topic.

## Localization

It contains launch file to make a visual odometry package work with the right parameters, and to start a kalman filter to combine the wheel odometry to the visual odometry, using parameters in the config folder.

## Map_server

This package overwrites the original map_server from the ROS navigation stack. It is upgraded with probabilistic map management.

* To save a probabilistic map, simply set a --occ threshold inferior to the --free threshold. For example:

```
rosrun map_server map_saver -f "/path_to_workspace/catkin_ws/src/fc_thesis/sensors/maps/$(date -Iseconds)" map:="rtabmap/grid_prob_map" --occ 1
```

* Loading your probabilistic map is done the same way as a trinary one:

```
rosrun map_server map_server /path/to/your/map_file.yaml
```

## My_dwb_critics

The dwb_local_planner package from the robot_navigation stack can run a local planner that all its features registered as plugins. Thus one can "easily" modify or add new features by creating a new plugin and changing a few things in a parameter.json file. This package holds my very own critic plugins

## Config examples

This folder showcases a few config files as examples, and to list the parameters that can be used. For a summary of some of the parameters available to configure your own planner using the robot_navigation stack, see the ROBOT_NAV_PARAMETERS.md

## Nav_utils

This package holds several utilities such as a keyboard controller tool, visuals and tools for debugging and analysis.

The README in this package gives details about the visuals and the debugging tools and strategy.
