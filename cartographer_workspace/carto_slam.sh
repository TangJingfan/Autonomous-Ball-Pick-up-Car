#!/bin/bash
source /opt/ros/melodic/setup.bash
catkin_make_isolated --install --use-ninja
source install_isolated/setup.bash
roslaunch cartographer_ros demo_revo_lds.launch 
