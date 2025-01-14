#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <serial/serial.h>

serial::Serial ser;

void cmd_vel_call_back(const geometry_msgs::Twist::ConstPtr &msg) {
  double linear_x = msg->linear.x;
  double angular_z = msg->angular.z;
}

int main(int argc, char **argv) {
  ros::init(argc, argc, "nav_driver");
  ros::NodeHandle nh;

  try {
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

  ros::Subscriber sub = nh.subscribe("/cmd_vel", 10, cmd_vel_call_back);

  ros::spin();

  ser.close();
  return 0;
}