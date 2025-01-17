#ifndef ABP_PLANNER_H
#define ABP_PLANNER_H

#include <nav_core/base_local_planner.h>
#include <ros/ros.h>

namespace abp_planner {
class ABPPlanner : public nav_core::BaseLocalPlanner {
public:
  ABBPlanner();
  ~ABBPlanner();

  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  bool isGoalReached();
};

} // namespace abp_planner
#endif