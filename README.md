# Autonomous-Ball-Pick-up-Car
This is a project regarding designing an autonomous ball pick-up mecanum-wheeled car.

## Structure 

All structure files are stored in `structure`. All part of the picker is named as `<name>-<manufacture_methon>_<date>.SLDPRT`. The assembly is named as `picker_assembly_20241130.SLDASM`.

## How to Run

### Lidar Slam

```sh
cd ~/Autonomous-Ball-Pick-up-Car/workspace
catkin_make
source /opt/ros/melodic/setup.bash
source devel/setup.bash
cd ./src/rplidar_ros/launch
roslaunch rplidar_ros view_rplidar.launch
```