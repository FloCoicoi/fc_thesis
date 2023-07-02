# My_dwb_critics

This package holds the critic plugins I implemented to be used with the dwb_local_planner from the robot_navigation stack.

## DynamicAvoidanceCritic

This critic subscribes to the /obstacles topic, using messages from the obstacle_detector third_party package. Given a trajectory to score, this critic iterates through every obstacles
