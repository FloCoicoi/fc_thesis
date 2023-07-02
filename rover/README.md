# README rover_

This folder holds packages needed to setup the rover and enable it to use the navigation and perception functionalities provided by the other packages.

## rover_controller

Is required to manage the commands provided by the navigation stack. It subscribes to the cmd_vel topic. In the case of the rover, the controller part is provided by the constructors in third_party/roverrobotics_ros1. Thus we only need to publish the cmd_vel messages to the topic cmd_vel/managed

## rover_nav

Contains the config files needed by the move_base package, and the launch file to read the config files and launch appropriate nodes for autonomous navigation.

## rover_tf_publisher

Contains scripts and launch files to publish the transform defining the position of the sensors on the robot. It also needs to publish the odom topic and transform, but the odometry is published by third_party/roverrobotics_ros1 already, and the odom transform is given by the EKF node launched with localization.launch
