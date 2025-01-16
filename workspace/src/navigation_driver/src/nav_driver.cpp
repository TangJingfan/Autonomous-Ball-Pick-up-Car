#include <algorithm>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>

serial::Serial ser;

std::string generate_control_command(double linear, double angular) {
  // init command
  std::string command = "<0,0,0,0>";

  // step 1. set parameters of the car (unit: m)
  double width = 0.24;
  double factor = 100.0 / pow(0.36, 2);

  // step 2. calculate voltage as double
  double v_left = factor * pow(linear, 2) - (angular * 80.0);
  double v_right = factor * pow(linear, 2) + (angular * 80.0);

  // step 3. apply minimum voltage or other constraints
  int voltage_left = static_cast<int>(v_left);
  int voltage_right = static_cast<int>(v_right);

  // step 4. set command
  command = "<" + std::to_string(voltage_left) + "," +
            std::to_string(voltage_right) + "," + std::to_string(voltage_left) +
            "," + std::to_string(voltage_right) + ">";

  return command;
}

// send info to arduino
void cmd_vel_call_back(const geometry_msgs::Twist::ConstPtr &msg) {
  // linear velocity
  double linear_x = msg->linear.x;
  // angular velocity
  double angular_z = msg->angular.z;

  std::string command = generate_control_command(linear_x, angular_z);

  try {
    ROS_INFO("linear velocity: %.2f; angular velocity: %.2f", linear_x,
             angular_z);
    ser.write(command);
    ROS_INFO("Sent Command: %s", command.c_str());
    ser.flushOutput();
  } catch (serial::IOException &e) {
    ROS_ERROR("Unable to send data to Arduino.");
  }
}

int main(int argc, char **argv) {
  // init ros node
  ros::init(argc, argv, "nav_driver");
  ros::NodeHandle nh;

  try {
    // init serial communication
    ser.setPort("/dev/arduino");
    ser.setBaudrate(115200);
    serial::Timeout to = serial::Timeout::simpleTimeout(1000);
    ser.setTimeout(to);
    ser.open();
  } catch (serial::IOException &e) {
    ROS_ERROR("Unable to open port.");
    return -1;
  }

  if (ser.isOpen()) {
    ROS_INFO("Serial Port initialized.");
  } else {
    return -1;
  }

  // subscribe /cmd_vel topic
  ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, cmd_vel_call_back);

  ros::spin();

  ser.close();
  return 0;
}
