#!/bin/bash

sudo apt install -y \
        ros-noetic-robot-localization \
        ros-noetic-ros-numpy \
        ros-noetic-twist-mux \
        ros-noetic-realsense2-camera \
        ros-noetic-rtabmap-ros \
        ros-noetic-pcl-ros \
        ros-noetic-geometry2 \
        ros-noetic-teleop-twist-keyboard \
        ros-noetic-navigation \
        ros-noetic-robot-navigation
cd ~/
mkdir library/
cd library/
git clone https://github.com/RoverRobotics/librover
cd librover/
cmake .
make
sudo make install
sudo apt-get install libsdl-image1.2-dev
sudo apt-get install libsdl-dev
