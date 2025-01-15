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

  // step 2. calculate velocity of left wheels and right wheels
  double v_left = linear - (angular * width) / 2;
  double v_right = linear + (angular * width) / 2;

  // step 3. set corresponding voltage
  int voltage_left = floor(v_left / 0.5 * 255);
  int voltage_right = floor(v_left / 0.5 * 255);
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
    ser.write(command);
    std::cout << "Sent Command: " << command << std::endl;
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
  ros::Subscriber sub = nh.subscribe("/cmd_vel", 18, cmd_vel_call_back);

  ros::spin();

  ser.close();
  return 0;
}
