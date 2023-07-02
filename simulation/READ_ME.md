# Simulation READ_ME

The simulation folder contains launches and configs to try the rover and the autonomous navigation in a simulated world.

## How to add a new simulation world

You need a folder named world_name and containing:
* A world_name.world file
* A models folder which may contain sub folders with the .sdf or .model files used by your new world, and other data such as a mesh folder

Then make sure to do the following:

```
export GAZEBO_RESOURCE_PATH=~/path_to_your_folder/world_name
export GAZEBO_MODEL_PATH=~/path_to_your_folder/world_name/models
```

In fc_thesis, the gazebo world resources are kept under simulation/worlds. So for example with the bookstore world you should run:

```
export GAZEBO_RESOURCE_PATH=~/catkin_ws/src/fc_thesis/simulation/worlds/bookstore
export GAZEBO_MODEL_PATH=~/catkin_ws/src/fc_thesis/simulation/worlds/bookstore/models
```

You can add these export lines in your bashrc if you wish.

## How to launch the simulation in a gazebo world

Once your world folder and the gazebo paths are set up, just launch the simulation with your world_name argument (for example bookstore):

```
roslaunch simulation simulation.launch world_name:="world_name"
```

## Make a map of a new world

If you added a new world and it has a map you can just put it in the right folder. But if you don't have one you can proceed with the gazebo_ros_2Dmap_plugin: https://github.com/marinaKollmitz/gazebo_ros_2Dmap_plugin. To do so follow the steps:

 * First insert these lines in your world file, anywhere between the \<world> and \</world>:

```
<plugin name='gazebo_occupancy_map' filename='libgazebo_2Dmap_plugin.so'>
    <map_resolution>0.1</map_resolution> <!-- in meters, optional, default 0.1 -->
    <map_height>0.3</map_height>         <!-- in meters, optional, default 0.3 -->
    <map_size_x>10</map_size_x>          <!-- in meters, optional, default 10 -->
    <map_size_y>10</map_size_y>          <!-- in meters, optional, default 10 -->
    <init_robot_x>0</init_robot_x>          <!-- x coordinate in meters, optional, default 0 -->
    <init_robot_y>0</init_robot_y>          <!-- y coordinate in meters, optional, default 0 -->
</plugin>
```

The plugin will "scan" your simulated world at the "map_height" height to produce the map. You might want to make sure this is the same as the camera's height of your robot. Also check the map dimensions and resolution to fit your world and needs.

* Then, in a terminal, run `roscore` and in a second tab, export your gazebo paths if not already done, and launch your world:

Begin with exporting the path to your new world:

```
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/user_name/catkin_ws/src/fc_thesis/simulation/worlds/world_name
```

And to the attached models if any:

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/user_name/catkin_ws/src/fc_thesis/simulation/worlds/world_name/models
```

* You can now launch the gazebo world (do not forget the ".world"):

```
roslaunch gazebo_ros empty_world.launch world_name:="world_name.world"
```

* And to generate the map, call the plugin's service:

```
rosservice call /gazebo_2Dmap_plugin/generate_map
```

* Finally, save your map using the following command, or executing the save_map script:

```
rosrun map_server map_saver -f "map_file_name" map:="map_topic"
```
or
```
source catkin_ws/src/fc_thesis/simulation/scripts/save_map.sh
```

The later will use the date and time to name your map, and place it in the simulation/maps folder.

## Run autonomous navigation in the simulation

Now you're all set up, let's run the navigation stack with the right map! First launch the simulation in the gazebo world you want to try (as taught above). Then launch the navigation stack using the right map:

```
roslaunch simulation move_base.launch map_name:="map_name"
```

* In another terminal, you can publish goals to see how the rover reacts. You can also use the "Publish 2D goal" button in rviz to get the same effect:

```
rostopic pub -1 /goal geometry_msgs/Pose '{position: {x: 0.5, y: 0.0, z: 0.0}, orientation: {x: 0.0,y: 0.0,z: 0.0,w: 1.0}}'
```

* Or you can publish velocity commands directly:

```
rostopic pub -r 5 /cmd_vel/managed geometry_msgs/Twist '{linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}'
```

* To teleoperate the rover, run the following. Then to send commands to your simulated rover just type the right keys in that same terminal:

```
rosrun utils teleop_twist_keyboard.py
```

* You can spawn moving obstacles at any location. To spawn several obstacles make sure to give a different "model" argument. There are randomly moving obstacles and teleoperated ones. Teleoperating such obstacles works the same as teleoperating the robot, just make sure you type your keys in the right terminal:

```
roslaunch simulation random_moving_box.launch model_name:="obst1" x:="2.0" y:="2.0"

or

roslaunch simulation teleoperable_obstacle.launch model_name:="teleobst1" x:="-2.0" y:="-2.0"
```
