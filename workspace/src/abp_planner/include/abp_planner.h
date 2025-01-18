#ifndef ABP_PLANNER_H
#define ABP_PLANNER_H

#include <nav_core/base_local_planner.h>
#include <ros/ros.h>

namespace abp_planner {
// local planner is packed as a class
// it must inherit from BaseLocalPlanner
class ABPPlanner : public nav_core::BaseLocalPlanner {
public:
  // constructor and destructor
  ABPPlanner();
  ~ABPPlanner();
  // main member function of the local planner
  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  bool isGoalReached();
};

} // namespace abp_planner
#endif