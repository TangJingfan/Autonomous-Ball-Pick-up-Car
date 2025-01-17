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
int target_index_;

void ABPPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                            costmap_2d::Costmap2DROS *costmap_ros) {
  tf_listener_ = new tf::TransformListener();
  ROS_WARN("Use abp local planner");
}
std::vector<geometry_msgs::PoseStamped> global_plan_;

bool ABPPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
  global_plan_ = plan;
  target_index_ = 0;
  return true;
}

bool ABPPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  geometry_msgs::PoseStamped target_pose;
  for (int i = target_index_; i < global_plan_.size(); i++) {
    geometry_msgs::PoseStamped pose_base;
    global_plan_[i].header.stamp = ros::Time(0);
    tf_listener_->transformPose("base_link", global_plan_[i], pose_base);
    double dx = pose_base.pose.position.x;
    double dy = pose_base.pose.position.y;
    double dist = sqrt(dx * dx + dy * dy);

    if (dist > 0.2) {
      target_pose = pose_base;
      target_index = i;
      ROS_WARN("Next target: %d, distance: %.2f", target_index_, dist);
      break;
    }

    if (target_index_ = global_plan_.size() - 1) {
      target_pose = pose_base;
    }
  }
  // 1.5 and 5 are proportion constant.
  // they need to be adjusted to correspond real car.
  cmd_vel.linear.x = target_pose.pose.position.x * 1.5;
  cmd_vel.angular.z = target_pose.pose.position.y * 5;

  cv::Mat plan_image(600, 600, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < global_plan_.size(); i++) {
    geometry_msgs::PoseStamped pose_base;
    global_plan_[i].header.stamp = ros::Time(0);
    tf_listener_->transformPose("base_link", global_plan_[i], pose_base);
    int cv_x = 300 - pose_base.pose.position.y * 100;
    int cv_y = 300 - pose_base.pose.position.x * 100;
    cv::circle(plan_image, cv::Point(cv_x, cv_y), 1, cv::Scalar(255, 0, 255));
  }
  cv::circle(plan_image, cv::Point(300, 300), 15, cv::Scalar(0, 255, 0));
  cv::line(plain_image, cv::Point(65, 300), cv::Point(510, 300),
           cv::Scalar(0, 255, 0), 1);
  cv::line(plain_image, cv::Point(300, 45), cv::Point(300, 555),
           cv::Scalar(0, 255, 0), 1);

  cv::namedWindow("Plan");
  cv::imshow("Plan", plan_image);
  cv::waitKey(1);
  // because i am working on a real robot, the real robot needs to move.
  // cmd_vel.angular.z = 1.0;
  return true;
}

bool ABPPlanner::isGoalReached() { return false; }
} // namespace abp_planner
