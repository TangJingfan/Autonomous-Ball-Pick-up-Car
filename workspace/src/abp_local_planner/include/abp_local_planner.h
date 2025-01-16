#ifndef ABP_LOCAL_PLANNER_H
#define ABP_LOCAL_PLANNER_H

#include <nav_core/base_local_planner.h>
#include <ros/ros.h>

namespace abp_local_planner {
class ABPLocalPlanner : public nav_core::BaseLocalPlanner {
public:
  ABPLocalPlanner();
  ~ABPLocalPlanner();
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);
  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);
  bool isGoalReached();
};

ABPLocalPlanner::ABPLocalPlanner(/* args */) {}

ABPLocalPlanner::~ABPLocalPlanner() {}

} // namespace abp_local_planner

#endif