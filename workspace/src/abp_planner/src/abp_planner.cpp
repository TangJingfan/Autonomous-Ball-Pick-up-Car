#include "abp_planner.h"
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(abp_planner::ABPPlanner, nav_core::BaseLocalPlanner)

namespace abp_planner {
ABPPlanner::ABBPlanner() { setlocale(LC_ALL.""); }
ABPPlanner::~ABBPlanner() {}

void ABPPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                            costmap_2d::Costmap2DROS *costmap_ros) {
  ROS_INFO("Use abp local planner");
}

bool ABPPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
  return true;
}

bool ABPPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  return true;
}

bool ABPPlanner::isGoalReached() { return false; }
} // namespace abp_planner
