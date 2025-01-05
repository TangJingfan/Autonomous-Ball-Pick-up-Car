#include <fstream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <yaml-cpp/yaml.h>

std::vector<std::vector<uint8_t>> loadPGM(const std::string &file_path) {
  cv::Mat image = cv::imread(file_path, cv::IMREAD_UNCHANGED);
  if (image.empty()) {
    throw std::runtime_error("Could not load PGM file: " + file_path);
  }

  std::vector<std::vector<uint8_t>> pgm_image;
  for (int i = 0; i < image.rows; ++i) {
    pgm_image.emplace_back(image.ptr<uint8_t>(i),
                           image.ptr<uint8_t>(i) + image.cols);
  }
  return pgm_image;
}

YAML::Node loadYAML(const std::string &file_path) {
  return YAML::LoadFile(file_path);
}

std::vector<std::array<float, 3>>
pgmToPointCloud(const std::vector<std::vector<uint8_t>> &pgm_image,
                const YAML::Node &yaml_data) {
  double resolution = yaml_data["resolution"].as<double>(0.05);
  std::vector<double> origin =
      yaml_data["origin"].as<std::vector<double>>({0.0, 0.0, 0.0});

  std::vector<std::array<float, 3>> points;
  int height = pgm_image.size();
  int width = pgm_image[0].size();

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      if (pgm_image[y][x] <
          255) { // Exclude white pixels (assumed to be empty space)
        float world_x = static_cast<float>(origin[0] + x * resolution);
        float world_y =
            static_cast<float>(origin[1] + (height - y - 1) * resolution);
        points.push_back({world_x, world_y, 0.0f}); // Z = 0 for 2D maps
      }
    }
  }
  return points;
}

void publishPointCloud(const std::vector<std::array<float, 3>> &points,
                       const std::string &frame_id, ros::Publisher &pub) {
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = frame_id;
  cloud.header.stamp = ros::Time::now();
  cloud.height = 1;
  cloud.width = points.size();
  cloud.is_bigendian = false;
  cloud.is_dense = true;

  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(points.size());

  sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
  for (const auto &point : points) {
    *iter_x = point[0];
    *(++iter_x) = point[1];
    *(++iter_x) = point[2];
    ++iter_x;
  }

  pub.publish(cloud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "pgm_to_pointcloud");
  ros::NodeHandle nh("~");

  std::string pgm_file, yaml_file, frame_id;
  nh.param("pgm_file", pgm_file, std::string("map.pgm"));
  nh.param("yaml_file", yaml_file, std::string("map.yaml"));
  nh.param("frame_id", frame_id, std::string("map"));

  try {
    ROS_INFO("Loading PGM file...");
    auto pgm_image = loadPGM(pgm_file);

    ROS_INFO("Loading YAML file...");
    auto yaml_data = loadYAML(yaml_file);

    ROS_INFO("Converting PGM to point cloud...");
    auto points = pgmToPointCloud(pgm_image, yaml_data);

    ROS_INFO("Publishing point cloud...");
    ros::Publisher pub =
        nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 10);
    ros::Rate rate(1);

    while (ros::ok()) {
      publishPointCloud(points, frame_id, pub);
      ros::spinOnce();
      rate.sleep();
    }
  } catch (const std::exception &e) {
    ROS_ERROR("Error: %s", e.what());
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
