#!/bin/bash

source install_isolated/setup.bash
rosservice call /finish_trajectory 0
rosservice call /write_state "{filename: '/home/nano/Autonomous-Ball-Pick-up-Car/workspace/src/map/map_info/map.pbstream'}"
rosrun cartographer_ros cartographer_pbstream_to_ros_map -map_filestem=/home/nano/Autonomous-Ball-Pick-up-Car/workspace/src/map/map_info/map.pbstream -pbstream_filename=/home/nano/Autonomous-Ball-Pick-up-Car/workspace/src/map/map_info/map.pbstream -resolution=0.05