#include "abp_planner.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <pluginlib/class_list_macros.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <vector>

// when you want to write a plugin for the ROS system, this declaration is a
// must tell the system what your plugin is and what will be replaced
PLUGINLIB_EXPORT_CLASS(abp_planner::ABPPlanner, nav_core::BaseLocalPlanner)

namespace abp_planner {
// constructor and destructor
ABPPlanner::ABPPlanner() { setlocale(LC_ALL, ""); }
ABPPlanner::~ABPPlanner() {}

// a transform listener for listening tf transform
tf::TransformListener *tf_listener_;
// an index to track global plan
int target_index_;
// a boolean variable to track pose
// true means robot needs to change it yaw angle
bool pose_adjusting_;
// a boolean to track whether reach goal
bool goal_reached_;

void ABPPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                            costmap_2d::Costmap2DROS *costmap_ros) {
  // create an instance for tf_listener
  tf_listener_ = new tf::TransformListener();
  // show we have started local planner
  ROS_WARN("Use abp local planner");
}

// a global variable to restore plan
std::vector<geometry_msgs::PoseStamped> global_plan_;

bool ABPPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) {
  // store data
  global_plan_ = plan;
  // restart tracking
  target_index_ = 0;
  // set pose adjusting as default false
  pose_adjusting_ = false;
  goal_reached_ = false;
  return true;
}

bool ABPPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  // a pose info to restore target status
  geometry_msgs::PoseStamped target_pose;

  // final index
  int final_index = global_plan_.size() - 1;
  geometry_msgs::PoseStamped pose_final;
  // add time stamp to pose
  global_plan_[final_index].header.stamp = ros::Time(0);
  // use function in tf listener to transform pose
  tf_listener_->transformPose("base_link", global_plan_[final_index],
                              pose_final);
  if (pose_adjusting_ == false) {
    double dx = pose_final.pose.position.x;
    double dy = pose_final.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);
    if (dist < 0.05) {
      pose_adjusting_ = true;
    }
  }

  if (pose_adjusting_ == true) {
    double final_yaw = tf::getYaw(pose_final.pose.orientation);
    ROS_WARN("Final adjusting, final_yaw = %.2f", final_yaw);
    cmd_vel.linear.x = pose_final.pose.position.x * 1.5;
    cmd_vel.angular.z = final_yaw * 0.5;

    if (abs(final_yaw) < 0.1) {
      ROS_WARN("Reach Goal!");
      cmd_vel.linear.x = 0;
      cmd_vel.angular.z = 0;
      goal_reached_ = true;
    }
  }

  for (int i = target_index_; i < global_plan_.size(); i++) {
    // a pose info to restore current status
    geometry_msgs::PoseStamped pose_base;
    // add time stamp to pose
    global_plan_[i].header.stamp = ros::Time(0);
    // use function in tf listener to transform pose
    tf_listener_->transformPose("base_link", global_plan_[i], pose_base);
    // calculate distance
    double dx = pose_base.pose.position.x;
    double dy = pose_base.pose.position.y;
    double dist = std::sqrt(dx * dx + dy * dy);

    // within this distance, we consider the robot has achieved goal
    if (dist > 0.2) {
      // update target info
      target_pose = pose_base;
      target_index_ = i;
      ROS_WARN("Next target: %d, distance: %.2f", target_index_, dist);
      break;
    }

    // when reach goal, final process
    if (i == global_plan_.size() - 1) {
      target_pose = pose_base;
    }
  }

  // update cmd_vel info
  // 1.5 and 5 are proportion constant.
  // they need to be adjusted to correspond real car.
  cmd_vel.linear.x = target_pose.pose.position.x * 1.2;
  cmd_vel.angular.z = target_pose.pose.position.y * 3;

  // add an image to plot curve
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
  cv::line(plan_image, cv::Point(65, 300), cv::Point(510, 300),
           cv::Scalar(0, 255, 0), 1);
  cv::line(plan_image, cv::Point(300, 45), cv::Point(300, 555),
           cv::Scalar(0, 255, 0), 1);

  cv::namedWindow("Plan");
  cv::imshow("Plan", plan_image);
  cv::waitKey(1);
  // because i am working on a real robot, the real robot needs to move.
  // cmd_vel.angular.z = 1.0;
  return true;
}

bool ABPPlanner::isGoalReached() { return goal_reached_; }
} // namespace abp_planner
