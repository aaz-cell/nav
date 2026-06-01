#include <ros/ros.h>

#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <Eigen/Dense>

#include <cmath>
#include <limits>
#include <string>

namespace {

double Clamp(double value, double low, double high) {
  return std::max(low, std::min(value, high));
}

}  // namespace

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcd_ground_align");
  ros::NodeHandle private_nh("~");

  std::string file_directory;
  std::string input_file_name;
  std::string output_file_name;
  double ground_seed_height = 0.6;
  double ransac_distance_threshold = 0.08;
  bool align_ground_to_zero = true;
  bool save_binary = true;

  private_nh.param("file_directory", file_directory, std::string("/home/"));
  private_nh.param("input_file_name", input_file_name, std::string("scans"));
  private_nh.param("output_file_name", output_file_name,
                   std::string("scans_aligned"));
  private_nh.param("ground_seed_height", ground_seed_height, 0.6);
  private_nh.param("ransac_distance_threshold", ransac_distance_threshold, 0.08);
  private_nh.param("align_ground_to_zero", align_ground_to_zero, true);
  private_nh.param("save_binary", save_binary, true);

  const std::string input_pcd = file_directory + input_file_name + ".pcd";
  const std::string output_pcd = file_directory + output_file_name + ".pcd";

  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_pcd, *input_cloud) == -1) {
    PCL_ERROR("Couldn't read file: %s\n", input_pcd.c_str());
    return -1;
  }

  if (input_cloud->points.empty()) {
    ROS_ERROR("Input cloud is empty: %s", input_pcd.c_str());
    return -1;
  }

  double z_min = std::numeric_limits<double>::max();
  for (size_t i = 0; i < input_cloud->points.size(); ++i) {
    z_min = std::min(z_min, static_cast<double>(input_cloud->points[i].z));
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr ground_seed_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(input_cloud);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_min + ground_seed_height);
  pass.filter(*ground_seed_cloud);

  if (ground_seed_cloud->points.size() < 100) {
    ROS_WARN("Ground seed cloud too small (%zu points), fallback to full cloud.",
             ground_seed_cloud->points.size());
    ground_seed_cloud = input_cloud;
  }

  pcl::SACSegmentation<pcl::PointXYZ> seg;
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(ransac_distance_threshold);
  seg.setInputCloud(ground_seed_cloud);

  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.segment(*inliers, *coefficients);

  if (inliers->indices.empty() || coefficients->values.size() < 4) {
    ROS_ERROR("Ground plane fitting failed for %s", input_pcd.c_str());
    return -1;
  }

  Eigen::Vector3d normal(coefficients->values[0], coefficients->values[1],
                         coefficients->values[2]);
  if (normal.norm() < 1e-6) {
    ROS_ERROR("Invalid ground normal from fitted plane.");
    return -1;
  }
  normal.normalize();
  if (normal.z() < 0.0) {
    normal = -normal;
  }

  const Eigen::Vector3d target(0.0, 0.0, 1.0);
  Eigen::Quaterniond rotation = Eigen::Quaterniond::FromTwoVectors(normal, target);
  Eigen::Affine3d transform = Eigen::Affine3d::Identity();
  transform.linear() = rotation.toRotationMatrix();

  // Shift the fitted ground plane close to z=0 after leveling so the later
  // z-band filter can consistently remove floor and ceiling layers.
  double ground_z_offset = 0.0;
  if (align_ground_to_zero) {
    Eigen::Vector3d ground_centroid = Eigen::Vector3d::Zero();
    size_t valid_ground_points = 0;
    for (size_t i = 0; i < inliers->indices.size(); ++i) {
      const int index = inliers->indices[i];
      if (index < 0 ||
          static_cast<size_t>(index) >= ground_seed_cloud->points.size()) {
        continue;
      }

      const pcl::PointXYZ &point = ground_seed_cloud->points[index];
      const Eigen::Vector3d rotated_point =
          rotation * Eigen::Vector3d(point.x, point.y, point.z);
      ground_centroid += rotated_point;
      ++valid_ground_points;
    }

    if (valid_ground_points > 0) {
      ground_centroid /= static_cast<double>(valid_ground_points);
      ground_z_offset = ground_centroid.z();
      transform.translation() = Eigen::Vector3d(0.0, 0.0, -ground_z_offset);
    } else {
      ROS_WARN("Ground inliers unavailable for z offset alignment, keep original z.");
    }
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*input_cloud, *aligned_cloud, transform.matrix());

  if (save_binary) {
    pcl::io::savePCDFileBinary(output_pcd, *aligned_cloud);
  } else {
    pcl::io::savePCDFileASCII(output_pcd, *aligned_cloud);
  }

  const double tilt_deg =
      std::acos(Clamp(normal.dot(target), -1.0, 1.0)) * 180.0 / M_PI;
  ROS_INFO("Aligned %s -> %s", input_pcd.c_str(), output_pcd.c_str());
  ROS_INFO("Ground plane: [%.6f, %.6f, %.6f, %.6f], inliers=%zu, tilt=%.3f deg",
           coefficients->values[0], coefficients->values[1],
           coefficients->values[2], coefficients->values[3],
           inliers->indices.size(), tilt_deg);
  ROS_INFO("Ground z offset after leveling: %.4f m%s", ground_z_offset,
           align_ground_to_zero ? " (shifted to z=0)" : " (shift disabled)");

  return 0;
}
