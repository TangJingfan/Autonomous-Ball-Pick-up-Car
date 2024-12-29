# Autonomous-Ball-Pick-up-Car

## Introduction

This project involves designing an autonomous ball pick-up car equipped with mecanum wheels. The car performs active Laser SLAM to map the room and then autonomously navigates to collect ping-pong balls from every corner.

## Structure 

All structural files are stored in the `structure` directory. The naming conventions for the parts and assembly files are as follows:

- Individual parts: `<name>-<manufacture_method>_<date>.SLDPRT`
- Assembly file: `picker_assembly_20241130.SLDASM`

## Laser SLAM

Follow the steps below to configure and run Laser SLAM:

```bash
cd ~/Autonomous-Ball-Pick-up-Car/workspace
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
cd ./src/rplidar_ros/launch
roslaunch rplidar_ros rplidar_slam.launch
```