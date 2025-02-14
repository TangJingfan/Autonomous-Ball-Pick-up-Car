#include "ros/ros.h"
#include "std_msgs/String.h"

void imu_corrected_callback(const sensor_msgs::Imu::ConstPtr &msg) {
  static ros::NodeHandle nh;
  static ros::Publisher pub =
      nh.advertise<sensor_msgs::Imu>("/imu_corrected_filtered", 10);
sensor_msgs::Imu::ConstPtr corrected_msg;
      msg.
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "imu_corrected_node");
  ros::NodeHandle nh;
  ros::Subscriber imu_corrected_sub =
      nh.subscribe("/imu/data", 1, imu_corrected_callback);

  ros::spin();
  return 0;
}