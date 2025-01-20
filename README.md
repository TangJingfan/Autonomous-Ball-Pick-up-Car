# Autonomous-Ball-Pick-up-Car

## Introduction

This project involves designing an autonomous ball pick-up car equipped with mecanum wheels. The car performs active Laser SLAM to map the room and then autonomously navigates to collect ping-pong balls from every corner.

## Structure

All structural files are stored in the `structure` directory. The naming conventions for the parts and assembly files are as follows:

- **Individual parts**: `<name>-<manufacture_method>_<date>.SLDPRT`
- **Assembly file**: `picker_assembly_20241130.SLDASM`

## Laser SLAM

SLAM is based on RPLIDAR A1. When performing SLAM, ensure all related nodes are activated.

- **Hector SLAM**: Follow these steps to configure and run Laser SLAM:

```bash
cd ~/Autonomous-Ball-Pick-up-Car/workspace
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
sudo chmod 666 /dev/laser
roslaunch rplidar_ros rplidar_slam.launch
```

- **Cartographer**: For better performance, Cartographer is used. Three terminal windows are required:

```bash
# Terminal 1
cd ~/Autonomous-Ball-Pick-up-Car/workspace
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
sudo chmod 666 /dev/laser
roslaunch rplidar_ros rplidar.launch

# Terminal 2
cd ~/Autonomous-Ball-Pick-up-Car/cartographer_workspace
./carto_slam.sh

# Terminal 3
cd ~/Autonomous-Ball-Pick-up-Car/cartographer_workspace
./map_save.sh

# Once the map is successfully saved, quit all terminals.
```

## Active SLAM

To improve mapping efficiency, a simple exploration algorithm is implemented. Follow these steps:

```bash
sudo chmod 666 /dev/arduino
roslaunch explore explore.launch
roslaunch map map_save.launch
```

## Odometry

The package `rf2o_laser_odometry` is primarily used for odometry. Execute the following command to run it:

```bash
roslaunch rf2o_laser_odometry rf2o_laser_odometry.launch
```

## Localization

### AMCL Localization

- The AMCL algorithm is applied for robot localization. Use the following command:

```bash
roslaunch point_to_point_nav amcl_test.launch
```

### Lidar-Based Localization

- Alternatively, a laser lidar-based localization method is provided. After mapping the target area, run the following command:

```bash
roslaunch point_to_point_nav point_to_point_nav.launch
```


## Keyboard Control

The chassis is controlled by an `ESP32-S3` board. To control movement using a keyboard, follow these steps:

1. Burn `controlled_move.ino` into the `ESP32-S3` board.
2. Open the serial monitor and set the baud rate to 115200.
3. Use the following keys to control movement:
   - `w`: Move forward
   - `a`: Turn left
   - `s`: Move backward
   - `d`: Turn right
   - `x`: Stop

## Navigation

A customized local planner is applied for the navigation stack. Follow these steps:

```bash
roslaunch point_to_point_nav point_to_point_nav.launch

# Use 2D pose estimate to calibrate the initial position.

# Use 2D Nav Goal to set the destination.
```