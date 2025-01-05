#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <nanoflann.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <vector>

struct Point2D {
  double x, y;
};