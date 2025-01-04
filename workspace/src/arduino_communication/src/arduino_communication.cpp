#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <serial/serial.h>
#include <sstream>
#include <vector>

// serial object
serial::Serial ser;

std::string
get_next_step_velocity(const sensor_msgs::LaserScan::ConstPtr &msg) {

  bool left_empty = true;
  bool forward_empty = true;
  bool right_empty = true;

  // read from laser and set obstacle
  std::vector<std::pair<float, float>> obstacles;
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    float distance = msg->ranges[i];
    float angle = msg->angle_min + i * msg->angle_increment;

    if (angle < 1.57 && angle > 0.77 && distance < 0.2 &&
        distance >= msg->range_min) {
      left_empty = false;
    }
    if (angle < 0.77 && angle > -0.77 && distance < 0.2 &&
        distance >= msg->range_min) {
      forward_empty = false;
    }
    if (angle > -1.57 && angle < -0.77 && distance < 0.2 &&
        distance >= msg->range_min) {
      right_empty = false;
    }
  }

  // std::cout << forward_empty << left_empty << right_empty << std::endl;

  if (forward_empty) {
    return "<100,100,100,100>";
  } else {
    if (right_empty) {
      return "<50,-50,-50,50>";
    } else if (!right_empty && left_empty) {
      return "<-25,25,25,-25>";
    } else {
      return "<0,0,0,0>";
    }
  }
}

// deal with LaserScan data
void laser_call_back(const sensor_msgs::LaserScan::ConstPtr &msg) {

  std::string chassis_command = get_next_step_velocity(msg);

  try {
    // send info to arduino
    ser.write(chassis_command);

    std::cout << chassis_command << std::endl;
    // ser.write("<100,100,100,100>");
    // flush output buffer
    ser.flushOutput();
    std::string received_msgs = ser.read();
    // ROS_INFO(received_msgs);
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
