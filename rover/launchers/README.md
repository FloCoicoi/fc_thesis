# Launchers

The launchers folder contains the launch files and other useful files (configs, vizuals, maps...) for real use cases, as opposed to the simulation use case.

## Offline slam

Some tasks can be run off-line, i.e. using recorded data to compute stuff on any computer, instead of computing in real time while the rover is running. Mapping for example can be done this way.

### 1) Record a rosbag (if you don't have one)

To run off-line mapping, you need to record a rosbag with the rover. Setup the environment you wish to map and place the rover at the desired map origin. Then launch the rover with recording option:

```
roslaunch launchers online_rover.launch record_rosbag:="true"
```

This will record the realsense images and depth, as well as the transforms and odometry, and the velocity commands. The rosbag will be located in /home/rover/.ros and will be named using the date and time of recording.

### 2) Offlin_slam with the realsense depth

Now that you possess a rosbag and have it stored in your computer, just launch the off-line mapping:

```
roslaunch launchers offline_slam.launch bag_file:="path/to/your/bag_file.bag"
```

The path to your rosbag must be absolute and end with ".bag" (such as /home/user_name/rosbags/bag_name.bag).

You can add the `use_scan:="true"` option to run a 2Dlidar estimate using the depth data, and use it for mapping.

### 3.1) Save a map

To save a map, use the following command or source the save_offline_map.sh script:

```
rosrun map_server map_saver -f "/path_to_workspace/catkin_ws/src/fc_thesis/launchers/maps/$(date -Iseconds)" map:="rtabmap/grid_prob_map" --free 40 --occ 90
```

You can replace the "$(date -Iseconds)" by any map name, and try different values for the free space and occupied space thresholds.

Be warry that the map_name.yaml file will refer to your .pgm file using the absolute path instead of the relative path. You might want to change that to just "map_name.pgm" and store both in the same folder

### 3.2) Save the probabilistic map

To save the probabilistic map instead of applying the free/occupied thresholds right away, just set a --occ threshold inferior to the --free threshold. For example:

```
rosrun map_server map_saver -f "/path_to_workspace/catkin_ws/src/fc_thesis/launchers/maps/$(date -Iseconds)" map:="rtabmap/grid_prob_map" --occ 1
```

Loading your probabilistic map is done the same way as a trinary one:

```
rosrun map_server map_server /path/to/your/map_file.yaml
```
