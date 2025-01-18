# Autonomous-Ball-Pick-up-Car

## Introduction

This project involves designing an autonomous ball pick-up car equipped with mecanum wheels. The car performs active Laser SLAM to map the room and then autonomously navigates to collect ping-pong balls from every corner.

## Structure 

All structural files are stored in the `structure` directory. The naming conventions for the parts and assembly files are as follows:

- Individual parts: `<name>-<manufacture_method>_<date>.SLDPRT`
- Assembly file: `picker_assembly_20241130.SLDASM`

## Passive Laser SLAM

- Hector SLAM is used in this project. Follow the steps below to configure and run Laser SLAM:

```bash
cd ~/Autonomous-Ball-Pick-up-Car/workspace
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
sudo chmod 666 /dev/laser
roslaunch rplidar_ros rplidar_slam.launch
```

- Cartographer is also applied for better performance. When using cartographer, three terminal windows are needed.

```bash
# open terminal 1
cd ~/Autonomous-Ball-Pick-up-Car/workspace
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
sudo chmod 666 /dev/laser
roslaunch rplidar_ros rplidar.launch

# open terminal 2
cd ~/Autonomous-Ball-Pick-up-Car/cartographer_workspace
./carto_slam.sh

# open terminal 3
cd ~/Autonomous-Ball-Pick-up-Car/cartographer_workspace
./map_save.sh

# Eventually, when map is successfully downloaded, quit all terminals.
```

## Active SLAM

To improve mapping efficiency, we implement a simple exploration algorithm. Follow the steps below.

```bash
sudo chmod 666 /dev/arduino
roslaunch explore explore.launch
roslaunch map map_save.launch
```

## Odometry

We mainly used the package rf2o_laser_odometry. Follow the command below to run odometry.

```bash
roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch
```
## Localization

- For simplicity, we apply AMCL algorithm to help robot localize itself. Follow the step below.

```bash
roslaunch point_to_point_nav amcl_test.launch
```

- However, we also provide a localization method by laser lidar. After mapping the targeted places, you can follow the step below.

```bash
roslaunch point_to_point_nav point_to_point_nav.launch
```

