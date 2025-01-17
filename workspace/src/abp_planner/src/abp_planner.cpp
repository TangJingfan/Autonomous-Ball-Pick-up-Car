#include "abp_planner.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS(abp_planner::ABPPlanner, nav_core::BaseLocalPlanner)

namespace abp_planner {
ABPPlanner::ABPPlanner() { setlocale(LC_ALL, ""); }
ABPPlanner::~ABPPlanner() {}

tf::TransformListener *tf_listener_;

void ABPPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                            costmap_2d::Costmap2DROS *costmap_ros) {
  tf_listener_ = new tf::TransformListener();
  ROS_WARN("Use abp local planner");
}
std::vector<geometry_msgs::PoseStamped> global_plan_;

bool ABPPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
  global_plan_ = plan;
  return true;
}

bool ABPPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < global_plan_.size(); i++) {
    geometry_msgs::PoseStamped pose_base;
    global_plan_[i].header.stamp = ros::Time(0);
    tf_listener_->transformPose("base_link", global_plan_[i], pose_base);
    int cv_x = 300 - pose_base.pose.position.y * 100;
    int cv_y = 300 - pose_base.pose.position.x * 100;
    cv::circle(plan_image, cv::Point(cv_x, cv_y), 1, cv::Scalar(255, 0, 255));
  }
  cv::namedWindow("Plan");
  cv::imshow("Plan", plan_image);
  cv::waitKey(1);
  cmd_vel.angular.z = 1.0;
  return true;
}

bool ABPPlanner::isGoalReached() { return false; }
} // namespace abp_planner
