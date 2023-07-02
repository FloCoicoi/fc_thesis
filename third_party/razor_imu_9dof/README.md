Official ROS Documentation
--------------------------
A much more extensive and standard ROS-style version of this documentation can be found on the ROS wiki at:

http://wiki.ros.org/razor_imu_9dof


Install and Configure ROS Package
---------------------------------
1) Install dependencies (for 3D visualization):

```bash
sudo apt-get install python-visual # For Ubuntu 16.04 and before
# For Ubuntu 18.04, install https://github.com/lebarsfa/visual/tree/bionic, or see https://github.com/ENSTABretagneRobotics/razor_imu_9dof/issues/47
sudo apt-get install python3-pip python3-wxgtk4.0 ; pip3 install vpython # From Ubuntu 20.04
```

2) Download code:

```bash
cd ~/catkin_workspace/src
git clone https://github.com/ENSTABretagneRobotics/razor_imu_9dof.git
cd ..
catkin_make
# For 3D visualization, from Ubuntu 20.04
cd src/razor_imu_9dof/nodes ; wget https://www.glowscript.org/docs/VPythonDocs/VPtoGS.py ; python3 VPtoGS.py ; cp -f Converted/display_3D_visualization.py display_3D_visualization.py ; cd ../../..
```

Install Arduino firmware
-------------------------
1) For SPX-15846 and DEV-16832 (OpenLog Artemis), you will need to follow the same instructions as for the ``OLA_IMU_Basics.ino`` sample from https://github.com/sparkfun/OpenLog_Artemis (i.e. get the drivers from https://learn.sparkfun.com/tutorials/how-to-install-ch340-drivers, install SparkFun Apollo3 boards in Arduino IDE as in https://learn.sparkfun.com/tutorials/installing-board-definitions-in-the-arduino-ide (add https://raw.githubusercontent.com/sparkfun/Arduino_Boards/master/IDE_Board_Manager/package_sparkfun_index.json to `File` → `Preferences` → `Additional Board Manager URLs`) and ensure you select `SparkFun Apollo3` → `SparkFun RedBoard Artemis ATP` as the board and install SparkFun ICM 20948 IMU Arduino library as in https://learn.sparkfun.com/tutorials/installing-an-arduino-library). For SEN-14001 (9DoF Razor IMU M0), you will need to follow the same instructions as for the default firmware on https://learn.sparkfun.com/tutorials/9dof-razor-imu-m0-hookup-guide and use an updated version of SparkFun_MPU-9250-DMP_Arduino_Library from https://github.com/lebarsfa/SparkFun_MPU-9250-DMP_Arduino_Library (an updated version of the default firmware is also available on https://github.com/lebarsfa/9DOF_Razor_IMU).

2) Open ``src/Razor_AHRS/Razor_AHRS.ino`` in Arduino IDE. Note: this is a [modified version](https://github.com/lebarsfa/razor-9dof-ahrs)
of Peter Bartz' original Arduino code (see https://github.com/ptrbrtz/razor-9dof-ahrs). 
Use this version - it emits linear acceleration and angular velocity data required by the ROS Imu message

3) Select your hardware here by uncommenting the right line in ``src/Razor_AHRS/Razor_AHRS.ino``, e.g.

```cpp
// HARDWARE OPTIONS
/*****************************************************************/
// Select your hardware here by uncommenting one line!
//#define HW__VERSION_CODE 10125 // SparkFun "9DOF Razor IMU" version "SEN-10125" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10736 // SparkFun "9DOF Razor IMU" version "SEN-10736" (HMC5883L magnetometer)
//#define HW__VERSION_CODE 14001 // SparkFun "9DoF Razor IMU M0" version "SEN-14001"
//#define HW__VERSION_CODE 15846 // SparkFun "OpenLog Artemis" version "SPX-15846"
#define HW__VERSION_CODE 16832 // SparkFun "OpenLog Artemis" version "DEV-16832"
//#define HW__VERSION_CODE 10183 // SparkFun "9DOF Sensor Stick" version "SEN-10183" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10321 // SparkFun "9DOF Sensor Stick" version "SEN-10321" (HMC5843 magnetometer)
//#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)
```

4) Upload Arduino sketch to the board


Configure
---------
In its default configuration, ``razor_imu_9dof`` expects a yaml config file ``my_razor.yaml`` with:
* USB port to use
* Calibration parameters

An example``razor.yaml`` file is provided.
Copy that file to ``my_razor.yaml`` as follows:

```bash
roscd razor_imu_9dof/config
cp razor.yaml my_razor.yaml
```

Then, edit ``my_razor.yaml`` as needed

Launch
------
Publisher only:

```bash
roslaunch razor_imu_9dof razor-pub.launch
```

Publisher and 3D visualization:

```bash
roslaunch razor_imu_9dof razor-pub-and-display.launch
```

Publisher only with diagnostics:

```bash
roslaunch razor_imu_9dof razor-pub-diags.launch
```

3D visualization only:

```bash
roslaunch razor_imu_9dof razor-display.launch
```


Calibrate
---------
For best accuracy, follow the tutorial to calibrate the sensors:

http://wiki.ros.org/razor_imu_9dof

An updated version of Peter Bartz's magnetometer calibration scripts from https://github.com/ptrbrtz/razor-9dof-ahrs is provided in the ``magnetometer_calibration`` directory.

Update ``my_razor.yaml`` with the new calibration parameters.

Dynamic Reconfigure
-------------------
After having launched the publisher with one of the launch commands listed above, 
it is possible to dynamically reconfigure the yaw calibration.

1) Run:

```bash
rosrun rqt_reconfigure rqt_reconfigure 
```
  
2) Select ``imu_node``. 

3) Change the slider to move the calibration +/- 10 degrees. 
If you are running the 3D visualization you'll see the display jump when the new calibration takes effect.

The intent of this feature is to let you tune the alignment of the AHRS to the direction of the robot driving direction, so that if you can determine that, for example, the AHRS reads 30 degrees when the robot is actually going at 35 degrees as shown by e.g. GPS, you can tune the calibration to make it read 35. It's the compass-equivalent of bore-sighting a camera.
