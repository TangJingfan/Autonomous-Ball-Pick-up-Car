#include <geometry_msgs/Twist.h>
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <sstream>

serial::Serial ser;

std::string generate_control_command(double linear_x, double angular_z) {
  std::string command = "<0,0,0,0>"; // init command

  // set command for chassis
  if (linear_x > 0) {
    // forward
    command =
        "<" + std::to_string(static_cast<int>(linear_x * 100)) + ",0,0,0>";
  } else if (linear_x < 0) {
    // backward
    command =
        "<0,0," + std::to_string(static_cast<int>(-linear_x * 100)) + ",0>";
  }

  if (angular_z > 0) {
    command = "<" + std::to_string(static_cast<int>(linear_x * 100)) + ",0," +
              std::to_string(static_cast<int>(angular_z * 100)) + ",0>";
  } else if (angular_z < 0) {
    command = "<" + std::to_string(static_cast<int>(linear_x * 100)) + ",0,0," +
              std::to_string(static_cast<int>(-angular_z * 100)) + ">";
  }
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
