#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <serial/serial.h>
#include <sstream>
#include <vector>

// serial object
serial::Serial ser;

// deal with LaserScan data
void laserCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
  // read from laser and set obstacle
  std::vector<std::pair<float, float>> obstacles;
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    float distance = msg->ranges[i];
    float angle = msg->angle_min + i * msg->angle_increment;
    // select useful data and set location in point cloud
    if (distance >= msg->range_min && distance <= msg->range_max) {
      float x = distance * cos(angle);
      float y = distance * sin(angle);
      obstacles.emplace_back(x, y);
    }
  }

  try {
    // send info to arduino
    ser.write("<100,100,100,100>");
    // flush output buffer
    ser.flushOutput();
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
  ros::init(argc, argv, "laser_to_arduino");

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
  ros::Subscriber sub = nh.subscribe("/scan", 7, laserCallback);

  // keep the node running
  ros::spin();

  ser.close();
  return 0;
}
