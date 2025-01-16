#include "abp_local_planner.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <vector>

PLUGINLIB_EXPORT_CLASS(abp_local_planner::ABPLocalPlanner,
                       nav_core::BaseLocalPlanner)

namespace abp_local_planner {
ABPLocalPlanner::ABPLocalPlanner() {}

ABPLocalPlanner::~ABPLocalPlanner() {}

void ABPLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                 costmap_2d::Costmap2DROS *costmap_ros){
    ROS_INFO("Use customized local planner!")}

std::vector<geometry_msgs::PoseStamped> global_plan_;

bool ABPLocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &plan) {
  global_plan_ = plan;
  return true;
}

bool ABPLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  cv::Mat plan_image(1000, 1000, CV_8UC3, cv::Scalar(0, 0, 0));
  for (int i = 0; i < global_plan_.size(); i++) {
    int cv_x = 500 - gloal_plan[i].pose.position.y * 100;
    int cv_y = 500 - gloal_plan[i].pose.position.x * 100;
    cv::circle(plan_image, cv::Point(cv_x, cv_y), 1, cv::Scalar(255, 0, 255));
  }

  cv::nameWindow("Plan");
  cv::imshow("Plan", plan_image);
  cv::waitKey(1);
  return true;
}

bool ABPLocalPlanner::isGoalReached() { return false; }
} // namespace abp_local_planner
