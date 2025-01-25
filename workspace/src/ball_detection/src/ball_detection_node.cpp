#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;

void img_callback() {}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ball_detection");
  ros::NodeHandle nh;
  ros::Subscriber img_sub = nh.Subscribe("/usb_cam/image_raw", 1, img_callback);
}