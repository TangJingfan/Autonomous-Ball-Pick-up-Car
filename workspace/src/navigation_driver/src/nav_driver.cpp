#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <serial/serial.h>
#include <sstream>
#include <vector>

int main(int argc, char **argv) {
  ros::init(argc, argc, "nav_driver");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/cmd_vel", 1, cmd_vel_call_back);
}