# Sensors and post-processing

This package contains launch files for the cameras, and post-processing tools. It is not rover-dependent as the sensor tools and cameras can be used on any robot.

## Depth to laserscan conversion

For a ground robot, even if the information it can perceive is 3D, its action space is 2D. Thus using 2D information to filter in the useful 3D information helps taking decisions in the 2D plane and drive the robot.

The laserscan_demo.launch file shows an example of how to use min_2dscan. For now, a nodelet must be launched first to make a pointcloud out of the rgb image and the aligned depth image. Then the min_2Dscan node selects the points within the robot's height, and for each angle it looks for the point with the minimal depth and adds this depth to the laserscan. Thus it behaves as an intelligent 2D laserscan that filters out the obstacles above the robot's height, and gets all those the robot may collide with.

## PCL_filter

This node takes as input a pointcloud given by the depth_image_proc package, from a depth image. The logic of the pcl_filter node uses the fact that the pcl obtained from the depth image is order row-wise from the top left corner of the image, so the points can be seen as a series of laserscans performed from the maera frame, with various orientation to scan from top to bottom.

This node uses the same logic as the min_scan algorithm to try to sum up the information of an interval of height by keeping only the closest point along the vertical line given for each yaw angle. Only now it is performed over several intervals of height. The result is thus a pointcloud made of a few 'scans', each summarizing the information of an interval of height around them.

This step is very useful as it can turn a pointcloud of about 300 000 points (for a 720x480 image) into a pointcloud of about 3 000 points to give 4 scans at different height. This is better than a laserscan.

## Obstacle_tracking

The obstacle_tracker.launch file is heavily inspired from the demo.launch file of the third_party/obstacle_detector package. It launches the nodes needed for obstacle velocity estimation. The nodes use a laserscan as input. It includes new parameters for the latest modifications of the obstacl_detector package.

To grasp the way the package work, please visit the authors' github page: https://github.com/tysik/obstacle_detector. The paper detailling their approach is available in their resources folder.

They use two nodes. A first one makes the 'extraction' as follows: given a laserscan or a PointCloud2 message, it makes clusters and tries to fit a line to each cluster. If that line ends up larger than the maximum size of a circle (parametrized) then it keeps it as a segment, otherwise it will be approximated as a circle that stand a bit behind the fitted line so that it cover very few of the space in front of the line, which is known to be free. The tracking node then attempts to match circles from one time to the next one.

## Obstacle filter

When an obstacle is seen by a LaserScan or a pointcloud, it is inscribed in the local planner's costmap whether it is tracked as a mobile obstacle or not. Thus the local planner will label as illicit any trajectory that crosses the current position of an obstacle. But if an obstacle is moving fast, and the robot could expect it to be gone quickly, then it should very well aim for the current obstacle's position. To allow that we thus need to ignore the tracked obstacles when updating the costmaps.

The node obstacle_filter broadcasts a filtered_scan whenever a scan message is received. The filtered_scan contains the information of the original scan where the tracked obstacles are removed. We must feed this filtered_scan to the global map so that it updates with the previously unkown static obstacles, but not with the moving ones. This prevents the global costmap from filling up with an occupied trail left after the moving obstacle's passage, and thus prevents awkward replanning attempts to circumnavigate these previous positions of an obstacle that is no longer there.

The obstacle filter is now able to use the same logic and apply it to pointclouds, such as pointclouds from the pcl_filter.
