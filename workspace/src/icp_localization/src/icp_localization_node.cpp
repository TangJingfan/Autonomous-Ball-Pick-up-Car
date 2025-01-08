#include <Eigen/Dense>
#include <cmath>
#include <fstream>
#include <geometry_msgs/PoseStamped.h>
#include <nanoflann.hpp>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <tf/transform_broadcaster.h>
#include <vector>

// define a 2D point struct
struct Point2D {
  double x, y;
};

// define a 2D point cloud struct
struct PointCloud {
  std::vector<Point2D> points;
  // get the size of point cloud
  inline size_t kdtree_get_point_count() const { return points.size(); }
  // idx is index; dim == 0 -> value of x; dim == 1 -> value of y
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    if (dim == 0) {
      return points[idx].x;
    } else {
      return points[idx].y;
    }
  }
  // bounding box; optimize the performance of kd tree
  template <class BBOX> bool kdtree_get_bbox(BBOX & /* bb */) const {
    return false;
  }
};

// define a kd tree structure; use euclidean distance as distance function
// kd tree will be performed on 2D point cloud
typedef nanoflann::
    KDTreeSingleIndexAdaptor</* distance function*/
                             nanoflann::L2_Simple_Adaptor<double, PointCloud>,
                             /*data structure*/ PointCloud, /* dim */ 2>
        my_kd_tree_t;

class ICPLocalization {
public:
  // constructor
  ICPLocalization(ros::NodeHandle &nh)
      : nh_(nh), map_received_(false), pose_x_(0.0), pose_y_(0.0),
        pose_theta_(0.0) {
    // subscribe /map topic; get map info
    map_sub_ = nh_.subscribe("/map", 1, &ICPLocalization::mapCallback, this);
    // subscribe /scan topic; get laser info
    scan_sub_ = nh_.subscribe("/scan", 1, &ICPLocalization::scanCallback, this);
    // publish /robot_pose; send out info related to robot pose
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("/robot_pose", 10);
    // init tf broadcaster
    tf::TransformBroadcaster tf_broadcaster_;
    // publish point cloud
    map_cloud_pub_ =
        nh_.advertise<sensor_msgs::PointCloud2>("/map_point_cloud", 1);
  }

private:
  ros::NodeHandle nh_;
  ros::Subscriber map_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher pose_pub_;
  tf::TransformBroadcaster tf_broadcaster_;
  ros::Publisher map_cloud_pub_;

  // the structure for point cloud of the map
  PointCloud map_cloud_;
  my_kd_tree_t *kd_tree_;
  bool map_received_;

  // robot pose
  double pose_x_;
  double pose_y_;
  double pose_theta_;

  void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg) {
    if (map_received_)
      return; // only use callback function once

    // convert grid map to point cloud
    convertMapToPointCloud(*msg, map_cloud_);

    // construct kd tree
    kd_tree_ = new my_kd_tree_t(2, map_cloud_,
                                nanoflann::KDTreeSingleIndexAdaptorParams(10));
    kd_tree_->buildIndex();

    // receive map
    map_received_ = true;
    ROS_INFO("Map received and converted to point cloud.");
  }

  void scanCallback(const sensor_msgs::LaserScan::ConstPtr &msg) {
    // check whether receive map
    if (!map_received_) {
      ROS_WARN("Map not received yet.");
      return;
    }
    // convert scan data to point cloud
    std::vector<Point2D> scan_points = convertScanToPointCloud(*msg);
    if (scan_points.empty()) {
      ROS_WARN("No valid scan points.");
      return;
    }

    // perform icp matching
    Eigen::Matrix3d transform = performICP(scan_points);
    // update pose
    updatePose(transform);
    // publish pose
    publishPose();
  }

  void convertMapToPointCloud(const nav_msgs::OccupancyGrid &map,
                              PointCloud &cloud) {
    // delete old cloud point info
    cloud.points.clear();
    // load basic map info
    double resolution = map.info.resolution;
    double origin_x = map.info.origin.position.x;
    double origin_y = map.info.origin.position.y;

    ROS_INFO("Map Info:");
    ROS_INFO("Resolution: %f", map.info.resolution);
    ROS_INFO("Width: %d, Height: %d", map.info.width, map.info.height);
    ROS_INFO("Origin: (%f, %f)", map.info.origin.position.x,
             map.info.origin.position.y);

    // PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(
        new pcl::PointCloud<pcl::PointXYZ>());

    for (unsigned int y = 0; y < map.info.height; ++y) {
      for (unsigned int x = 0; x < map.info.width; ++x) {
        int index = x + y * map.info.width;
        int8_t value = map.data[index];
        if (value >= 75) { // occupied point
          double map_x = origin_x + x * resolution;
          double map_y = origin_y + y * resolution;
          cloud.points.emplace_back(Point2D{map_x, map_y});

          // Add to PCL cloud
          pcl_cloud->points.emplace_back(pcl::PointXYZ(map_x, map_y, 0.0));
        }
      }
    }

    // Convert to sensor_msgs::PointCloud2
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*pcl_cloud, cloud_msg);
    cloud_msg.header.frame_id = "map";
    cloud_msg.header.stamp = ros::Time::now();

    // Publish point cloud
    map_cloud_pub_.publish(cloud_msg);
  }

  std::vector<Point2D>
  convertScanToPointCloud(const sensor_msgs::LaserScan &scan) {
    std::vector<Point2D> points;
    double angle = scan.angle_min;
    for (const auto &range : scan.ranges) {
      if (range >= scan.range_min && range <= scan.range_max) {
        double x = range * std::cos(angle);
        double y = range * std::sin(angle);
        points.emplace_back(Point2D{x, y});
      }
      angle += scan.angle_increment;
    }
    return points;
  }

  Eigen::Matrix3d performICP(std::vector<Point2D> source) {
    // init transform matrix as identity
    Eigen::Matrix3d total_transform = Eigen::Matrix3d::Identity();

    // set icp iteration times
    int max_iterations = 30;
    double tolerance = 1e-4;

    std::vector<Point2D> source_transformed = source;

    for (int iter = 0; iter < max_iterations; ++iter) {
      // find closest
      std::vector<Point2D> target_matched;
      for (const auto &point : source_transformed) {
        double query_pt[2] = {point.x, point.y};
        size_t ret_index;
        double out_dist_sqr;
        nanoflann::KNNResultSet<double> resultSet(1);
        resultSet.init(&ret_index, &out_dist_sqr);
        kd_tree_->findNeighbors(resultSet, &query_pt[0],
                                nanoflann::SearchParameters(10));
        target_matched.emplace_back(map_cloud_.points[ret_index]);
      }

      // calculate centroid
      Point2D centroid_source = computeCentroid(source_transformed);
      Point2D centroid_target = computeCentroid(target_matched);

      // centralize point cloud
      std::vector<Eigen::Vector2d> src_centered;
      std::vector<Eigen::Vector2d> tgt_centered;
      for (size_t i = 0; i < source_transformed.size(); ++i) {
        src_centered.emplace_back(
            Eigen::Vector2d(source_transformed[i].x - centroid_source.x,
                            source_transformed[i].y - centroid_source.y));
        tgt_centered.emplace_back(
            Eigen::Vector2d(target_matched[i].x - centroid_target.x,
                            target_matched[i].y - centroid_target.y));
      }

      // calculate rotation matrix
      Eigen::Matrix2d H = Eigen::Matrix2d::Zero();
      for (size_t i = 0; i < src_centered.size(); ++i) {
        H += src_centered[i] * tgt_centered[i].transpose();
      }

      Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU |
                                                   Eigen::ComputeFullV);
      Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();

      // prevent reflection
      if (R.determinant() < 0) {
        Eigen::Matrix2d V = svd.matrixV();
        V.col(1) *= -1;
        R = V * svd.matrixU().transpose();
      }

      // calculation translation vector
      Eigen::Vector2d t =
          Eigen::Vector2d(centroid_target.x, centroid_target.y) -
          R * Eigen::Vector2d(centroid_source.x, centroid_source.y);

      // current transform matrix
      Eigen::Matrix3d current_transform = Eigen::Matrix3d::Identity();
      current_transform.block<2, 2>(0, 0) = R;
      current_transform.block<2, 1>(0, 2) = t;

      // update transform
      total_transform = current_transform * total_transform;

      // apply transform
      for (auto &point : source_transformed) {
        Eigen::Vector3d pt(point.x, point.y, 1.0);
        Eigen::Vector3d pt_transformed = current_transform * pt;
        point.x = pt_transformed(0);
        point.y = pt_transformed(1);
      }

      // calculate error
      double rmse = 0.0;
      for (size_t i = 0; i < source_transformed.size(); ++i) {
        double dx = source_transformed[i].x - target_matched[i].x;
        double dy = source_transformed[i].y - target_matched[i].y;
        rmse += dx * dx + dy * dy;
      }
      rmse = std::sqrt(rmse / source_transformed.size());

      // check convergence condition
      if (rmse < tolerance) {
        ROS_INFO("ICP converged at iteration %d with RMSE: %f", iter, rmse);
        break;
      }

      if (iter == max_iterations - 1) {
        ROS_WARN("ICP did not converge after %d iterations. Final RMSE: %f",
                 iter, rmse);
      }
    }

    return total_transform;
  }

  Point2D computeCentroid(const std::vector<Point2D> &points) {
    Point2D centroid = {0.0, 0.0};
    for (const auto &p : points) {
      centroid.x += p.x;
      centroid.y += p.y;
    }
    centroid.x /= points.size();
    centroid.y /= points.size();
    return centroid;
  }

  void updatePose(const Eigen::Matrix3d &transform) {
    // take out translation and rotation
    pose_x_ = transform(0, 2);
    pose_y_ = transform(1, 2);
    pose_theta_ = std::atan2(transform(1, 0), transform(0, 0));
  }

  void publishPose() {
    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "map";
    pose_msg.pose.position.x = pose_x_;
    pose_msg.pose.position.y = pose_y_;
    pose_msg.pose.position.z = 0.0;
    tf::Quaternion q;
    q.setRPY(0, 0, pose_theta_);
    pose_msg.pose.orientation.x = q.x();
    pose_msg.pose.orientation.y = q.y();
    pose_msg.pose.orientation.z = q.z();
    pose_msg.pose.orientation.w = q.w();

    pose_pub_.publish(pose_msg);

    // publish TF
    tf::Transform transform_tf;
    transform_tf.setOrigin(tf::Vector3(pose_x_, pose_y_, 0.0));
    transform_tf.setRotation(q);
    tf_broadcaster_.sendTransform(tf::StampedTransform(
        transform_tf, ros::Time::now(), "map", "base_link"));
  }
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "icp_localization_node");
  ros::NodeHandle nh;

  ICPLocalization icp_localization(nh);

  ros::spin();

  return 0;
}