#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <serial/serial.h>
#include <sstream>
#include <vector>

// serial object
serial::Serial ser;

// deal with LaserScan data
void laser_call_back(const sensor_msgs::LaserScan::ConstPtr &msg) {
  try {
    // send info to arduino
    ser.write("<100,100,100,100>");
    // flush output buffer
    ser.flushOutput();
    ROS_INFO("Sent!");
    std::string received_msgs = ser.read();
    // ROS_INFO("Sent to Arduino: %s", received_msgs.c_str());
    std::cout << received_msgs << std::endl;
    // ros::Duration(0.1).sleep(); // Sleep for 100 milliseconds
  } catch (serial::IOException &e) {
    ROS_ERROR("Unable to send data to Arduino.");
  }
}

int main(int argc, char **argv) {
  // init node
  ros::init(argc, argv, "arduino_communication");

  // init a node handle
  ros::NodeHandle nh;

  try {
    // init serial
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

  // subscribe topic "/scan"
  ros::Subscriber sub = nh.subscribe("/scan", 7, laser_call_back);

  // keep the node running
  ros::spin();

  ser.close();
  return 0;
}
