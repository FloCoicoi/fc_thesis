
# fc_thesis

Follow this README to install everything you need to get started and to run a quick demo.
This install was tested on Ubuntu 20.04 with ROS Noetic and Python3.

## Prerequisites

1. If ROS Noetic is not installed, please do so : http://wiki.ros.org/noetic/Installation/Ubuntu

2. It is recommended that you add `source /opt/ros/noetic/setup.bash` to your `.bashrc`:

```
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

3. Install ROS dependencies (please note this will add in your home a folder named /library related to the rover used in this thesis):

```
source INSTALL_DEPENDENCIES.sh
```

4. If you which to be able to remote control the rover with a ps4 controller, install `ds4drv`:

```
git clone https://github.com/naoki-mizuno/ds4drv --branch devel
cd ds4drv
mkdir -p ~/.local/lib/python3.8/site-packages
python3 setup.py install --prefix ~/.local
sudo cp udev/50-ds4drv.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```


## Building

1. Once ROS Noetic is installed, start by creating a new catkin workspace:

```
mkdir -p ~/catkin_ws/src
```

2. Clone fc_thesis in the workspace:

```
cd ~/catkin_ws/src/ && git clone git@github.com:FloCoicoi/fc_thesis.git
```

3. Build the workspace:

```
cd ~/catkin_ws/ && catkin_make
```

Depending on your python setup you might need your first catkin_make to specify using python3:

```
cd ~/catkin_ws/ && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3
```

4. Don't forget you need to source the workspace you wish to use before trying to launch anything:

```
source devel/setup.bash
```

If this is your only catkin workspace or if it is the main one you will be using, you can add the previous line at the end of `.bashrc` as follow:

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

5. Extra setup for simulation:

You might want to run some preset worlds. To do so you need to specify the path to your worlds folder. So to avoid doing it manually you can add this line to your bashrc:

```
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/fc_thesis/simulation/worlds/bookstore:~/catkin_ws/src/fc_thesis/simulation/worlds/large_room
export GAZEBO_MODEL_PATH=~/catkin_ws/src/fc_thesis/simulation/worlds/bookstore/models
```

If you want to teleoperate the rover in simulation, you need to run the following install:

```
sudo apt-get install ros-noetic-teleop-twist-keyboard
```

## fc_thesis dependencies presentation

You should now have a new catkin_ws directory in your `/home`. It contains the directories build, devel and src. Inside src there is the fc_thesis directory, which contains:

    src
    ├── my_navigation: Navigation and control stuff that makes the robot move autonomously:
        ├── dynobst_layer: Work in progress to make a costmap_2d layer able to aggregate information from a pointcloud in a better way than simple projection on the 2d plan.
        ├── getting_started: Work in progress meant to provide examples to help setup a robot with the planner designed in this thesis. The repository already includes other packages to make it work with the simulated rover.
        ├── goal_publisher: Package to publish move_base goals
        ├── localization: This package contains mainly a launch file to start the odometry nodes.
        ├── map_server: This package is based on the ROS navigation map_server package, and enables probabilistic maps loading and saving (WIP).
        ├── my_dwb_critics: Holds all the new critic plugins for the dwb_local_planner.
        ├── my_dwb_plugins: Contains the double trajectory generator.
        └── nav_utils: It contains debugging visualizers, and the teleop_twist_keyboard utility.
    ├── rover: Directory containing additional packages to make the rover work with the sensors and planners.
        ├── launchers: Launch files for the real rover, or for running the rover nodes offline while a rosbag plays. This package's README teaches you about running SLAM.
        ├── rover_controller: Nodes converting messages from cmd_vel topics published by the navigation stack into messages the robot can manage, under the cmd_vel/managed topic.
        ├── rover_nav: Contains configs and launch files to make the navigation work with that robot.
        ├── rover_tf_setup: Tf broadcasters needed by the navigation package.
    ├── sensors: Package containing launches for various cameras, and a pseudoLidar node to convert 3D pointclouds to planar laserscans.
    ├── simulation: This package contains everything required to work in a gazebo simulation. See internal README for more info.
    └── third_party: Third-party packages used in this thesis. Some are slightly modified for the needs of this thesis.


## Run the simulation

To try out the rover simulation you can run the following commands:

* In a first terminal:
```
roslaunch simulation simulation.launch
```

* In another terminal, initiate the autonomous navigation:
```
roslaunch simulation robot_nav.launch
```

A rviz window will now open. You can use it to publish a navigation goal and visualize some data (check out the nav_utils package and README!).

* To run some pre-built test demos, close the previous simulation (if any) and run any launch file from the simulation/launch/scenari folder:
```
roslaunch simulation obstacle_avoidance_front.launch
```

The simulation opens, you need to manually place the goal in front of the robot on the opposite side of the room. The obstacle will start moving when the robot does.

* Or run a series of randomly generated test cases:
```
python simulation/test.run_test.py
```

This will autonomously run several tests one after the other and store the results in the test_results file.

## Additional tools to run in simulation

* To teleoperate the rover, run the following. Then to send commands to your simulated rover just type the right keys in that same terminal (azerty keyboard):

```
rosrun nav_utils teleop_twist_keyboard.py
```

* You can spawn moving obstacles at any location. To spawn several obstacles make sure to give a different "model" argument. There are randomly moving obstacles and teleoperated ones. Teleoperating such obstacles works the same as teleoperating the robot, just make sure you type your keys in the right terminal:

```
roslaunch simulation random_moving_box.launch model_name:="obst1" x:="2.0" y:="2.0"

or

roslaunch simulation teleoperable_obstacle.launch model_name:="teleobst1" x:="-2.0" y:="-2.0"
```

## Run on real rover (same install on Ubuntu 20 embedded machine)

* To initiate every necessary nodes:

```
roslaunch launchers online_rover.launch
```

* Then to initiate autonomous navigation intelligence run the folllowing. You can specify a map_name, the default one is a map of the vb office:

```
roslaunch rover_nav move_base.launch

or

roslaunch rover_nav move_base.launch map_name:="map_name"
```

* Then you can use the ps4 remote controller to teleoperate the rover, or you can `rosrun utils teleop_twist_keyboard.py` through ssh or with a bluetooth keyboard, and command the rover just as in simulation. You can send goals using the terminal command through ssh or using a bluetooth keyboard, and you can also use the 2D Goal button on the Rviz window.

* Keep in mind that the ps4 controller will overwrite any other input. So have it at reach and be ready to make the rover turn on itself, to keep it on place in case it misbehaves.

## Other functionalities:

To learn more about the simulation functionalities, see the simulation README

To learn more about offline mapping of a real environment, see the launchers README

To learn more about probabilistic maps management, and other navigation stuff see the README in my_navigation

# Quick demo

for a quick demo after completing the install you can run
