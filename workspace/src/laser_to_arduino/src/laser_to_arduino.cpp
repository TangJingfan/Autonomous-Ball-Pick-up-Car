#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <serial/serial.h>
#include <sstream>

// serial object
serial::Serial ser;

// deal with LaserScan data
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // get min distance
  float min_distance =
      *std::min_element(msg->ranges.begin(), msg->ranges.end());

  // send info to arduino
  std::stringstream ss;
  ss << "w"
     << "\n";
  // ss << "distance:" << min_distance << "\n";
  std::string command = ss.str();

  try {
    ser.write(command);
    ser.flushOutput();
    ROS_INFO("Sent to Arduino: %s", command.c_str());
    ros::Duration(0.1).sleep(); // Sleep for 100 milliseconds
  } catch (serial::IOException &e) {
    ROS_ERROR("Unable to send data to Arduino.");
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "laser_to_arduino");
  ros::NodeHandle nh;

  // init serial
  try {
    ser.setPort("/dev/arduino"); // modify it to real serial
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

  // subscribe
  ros::Subscriber sub = nh.subscribe("/scan", 10, laserCallback);

  // keep running
  ros::spin();

  ser.close();
  return 0;
}
