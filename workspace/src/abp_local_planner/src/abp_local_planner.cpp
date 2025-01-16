#include "abp_local_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(abp_local_planner::ABPLocalPlanner,
                       nav_core::BaseLocalPlanner)

namespace abp_local_planner {
ABPLocalPlanner::ABPLocalPlanner() {}

ABPLocalPlanner::~ABPLocalPlanner() {}

void ABPLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                                 costmap_2d::Costmap2DROS *costmap_ros) {
  ROS_INFO("Use customized local planner!")
}

bool ABPLocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &plan) {
  return true;
}

bool ABPLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  return true;
}

bool ABPLocalPlanner::isGoalReached() { return false; }
} // namespace abp_local_planner
