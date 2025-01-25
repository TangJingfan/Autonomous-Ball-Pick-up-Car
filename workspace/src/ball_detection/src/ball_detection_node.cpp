#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

using namespace cv;

void img_callback(const sensor_msgs::Image msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::exception &e) {
    ROS_ERROR("cv bridge exception: %s", e.what());
    return;
  }
  Mat img_original = cv_ptr->image;
  imshow("RGB", img_original);
  waitkey(1);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "ball_detection");
  ros::NodeHandle nh;
  ros::Subscriber img_sub =
      nh.Subscribe("/usb_cam/image_rect_color", 1, img_callback);

  namedWindow("RGB");
  ros::spin();
}