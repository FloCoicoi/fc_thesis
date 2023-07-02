# Nav utilities

## Teleop_twist_keyboard

The script teleop_twist_keyboard.py allows you to publish cmd_vel through key presses. Thus you can drive the robot just as you would do in a video game. The key mapping is for azerty keyboards.

By remapping the topic published by this node, one can take inspiration from this script to teleoperate various things. For example in simulation, teleoperated obstacles.

## Vizuals

There are currently three scripts that helps with visualizing data for behavior analysis and debugging. They subscribe to some data published by the dwb_local_planner if the right parameters are set to true (see parameter description in the getting_started folder), and pose-process them to publish data such as point clouds to show on rviz.

### Trajectory_end_visualizer.py

This script subscribes to the evaluation topic, and simply goes through each generated trajectory and stores their end points to publish them as a PoseArray. This is used to see how the trajectory generator covers the 2D space. Ideally we want to see points in many positions and orientations, and avoid sampling several trajectories landing on the same spot.

### critic_costmap_visualizer.py

Some critics use a private costmap that they update once before planning so that computing a trajectory score comes down to looking at the value of the cell where the trajectory lands, or to add up the cell values along the trajectory.

If the parameter publish_cost_grid_pc is true, then these critics will publish their costmap, and the critic_costmap_visualizer.py helps post-processing these point clouds. For example, occupied space might have a much larger value than any other cell. Thus the automated rviz display will have a very bad contrast and there would be no difference between the cells with smaller values. To work around that one can simply filter out the points above a certain threshold, so that the smaller values have more contrast.

### scoretrajectory_visualizer.py

This one subscribes to the evaluation topic and outputs a pointcloud.

Each point has a value for every critic score, and for the total. In rviz it is possible to choose which "channel" to display, thus choosing which critic score to show.

There are several ways of using this tool. If the local planner parameter short_circuit_trajectory_evaluation is set to true, the critics at the end of the list will be called for very few points only. But a point cloud requires that each point has a value for every channel. Thus to set the missing values relted to a critic (when it has not been called for a trajectory) to the maximum value that critic gave during this planning loop. Thus the score showed for that critic on trajectories it did not score are going to be the same color as the trajectory it scored the highest.

An other way of using this tool is to specify a critic name when running the script. For example:

```
rosrun nav_utils scoretrajectory_visualizer.py DynamicAvoidance
```

This command will run the visualizer to focus on this critic. Thus the visualizer will only show trajectories that have been scored by the specified critic.

Finally, one can set short_circuit_trajectory_evaluation to false and run the script without additional arguments. This way every trajectory will be scored during planning, and thus the visual will allow you to see everything.

## Super Analysis and Debug setup

There exist a magnifiscent visualization tool that allow the most fellow ROS users to monitor pretty much everything.

The rqt_bag command opens a window that lets you open a rosbag, choose the topics from that bag to publish, visualize what you want, play the bag data one step at a time or at reduced speed, to go back to a certain time or forward to a ceretain time...

And to make things even easier I made a little launch file named bag_viz.launch which does three simple things:
 * It sets the use_sim_time parameter to true
 * It opens the rqt_bag tool
 * It opens rviz with the specified rviz config file

The user must then:
 * Open the wanted rosbag
 * Right click -> Publish and select All, or at least the clock topic and the one they want.
 * Run other nodes in other terminal tabs alongside this rqt_bag tool to post-process some data

It is thus possible to record some autonomous navigation action, and afterwards examine it with all the time and details needed.

Running rqt_bag using the launch file requires some python libraries. If you mind installing them you'll have to run rqt_bag, set the use_sim_time to true and open the right rviz config file by you own means.

## Record a bag

To record a bag under simulation for analysis, please don't have vizualization tools running online, as you can use them offline during the analysis, and use the nav_utils record_sim.lauch to register only the useful topics (this is mainly good for ignoring the many useless image topics that gazebo produces).
