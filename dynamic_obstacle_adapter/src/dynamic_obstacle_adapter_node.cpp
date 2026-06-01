#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <tf2/utils.h>

#include <algorithm>
#include <cmath>
#include <vector>

namespace {

class DynamicObstacleAdapter {
 public:
  DynamicObstacleAdapter(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    LoadParams();

    map_sub_ = nh_.subscribe(static_map_topic_, 1, &DynamicObstacleAdapter::StaticMapCallback, this);
    localization_sub_ =
        nh_.subscribe(localization_topic_, 10, &DynamicObstacleAdapter::LocalizationCallback, this);
    scan_sub_ = nh_.subscribe(scan_topic_, 10, &DynamicObstacleAdapter::ScanCallback, this);
    merged_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(merged_map_topic_, 1, true);

    ROS_INFO("dynamic_obstacle_adapter ready: %s + %s + %s -> %s",
             static_map_topic_.c_str(), scan_topic_.c_str(), localization_topic_.c_str(),
             merged_map_topic_.c_str());
  }

 private:
  void LoadParams() {
    pnh_.param<std::string>("static_map_topic", static_map_topic_, "/prior_map");
    pnh_.param<std::string>("scan_topic", scan_topic_, "/scan");
    pnh_.param<std::string>("localization_topic", localization_topic_, "/localization");
    pnh_.param<std::string>("merged_map_topic", merged_map_topic_, "/isweep_planner/merged_map");
    pnh_.param<double>("obstacle_range_min", obstacle_range_min_, 0.05);
    pnh_.param<double>("obstacle_range_max", obstacle_range_max_, 8.0);
    pnh_.param<int>("obstacle_inflation_cells", obstacle_inflation_cells_, 1);
    pnh_.param<double>("publish_rate", publish_rate_hz_, 1.0);
    if (publish_rate_hz_ < 0.1) {
      publish_rate_hz_ = 0.1;
    }
  }

  void StaticMapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    static_map_ = *msg;
    have_static_map_ = true;
    PublishMergedMap(true);
  }

  void LocalizationCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_localization_ = *msg;
    have_localization_ = true;
  }

  void ScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    latest_scan_ = *msg;
    have_scan_ = true;
    PublishMergedMap(false);
  }

  bool WorldToGrid(double wx, double wy, int* gx, int* gy) const {
    if (!have_static_map_ || gx == nullptr || gy == nullptr) {
      return false;
    }

    const double origin_x = static_map_.info.origin.position.x;
    const double origin_y = static_map_.info.origin.position.y;
    const double resolution = static_map_.info.resolution;
    const int width = static_cast<int>(static_map_.info.width);
    const int height = static_cast<int>(static_map_.info.height);

    const int ix = static_cast<int>(std::floor((wx - origin_x) / resolution));
    const int iy = static_cast<int>(std::floor((wy - origin_y) / resolution));
    if (ix < 0 || iy < 0 || ix >= width || iy >= height) {
      return false;
    }

    *gx = ix;
    *gy = iy;
    return true;
  }

  void MarkOccupied(nav_msgs::OccupancyGrid* map, int gx, int gy) const {
    if (map == nullptr) {
      return;
    }

    const int width = static_cast<int>(map->info.width);
    const int height = static_cast<int>(map->info.height);
    for (int dx = -obstacle_inflation_cells_; dx <= obstacle_inflation_cells_; ++dx) {
      for (int dy = -obstacle_inflation_cells_; dy <= obstacle_inflation_cells_; ++dy) {
        const int x = gx + dx;
        const int y = gy + dy;
        if (x < 0 || y < 0 || x >= width || y >= height) {
          continue;
        }
        const size_t index = static_cast<size_t>(y * width + x);
        map->data[index] = std::max<int8_t>(map->data[index], 100);
      }
    }
  }

  void PublishMergedMap(bool force) {
    if (!have_static_map_ || !have_scan_ || !have_localization_) {
      return;
    }

    const ros::Time now = ros::Time::now();
    if (!force && last_publish_time_.isValid() &&
        (now - last_publish_time_).toSec() < (1.0 / publish_rate_hz_)) {
      return;
    }

    nav_msgs::OccupancyGrid merged_map = static_map_;
    merged_map.header.stamp = now;
    if (merged_map.header.frame_id.empty()) {
      merged_map.header.frame_id = "map";
    }

    const double robot_x = latest_localization_.pose.pose.position.x;
    const double robot_y = latest_localization_.pose.pose.position.y;
    const double robot_yaw = tf2::getYaw(latest_localization_.pose.pose.orientation);

    for (size_t i = 0; i < latest_scan_.ranges.size(); ++i) {
      const float range = latest_scan_.ranges[i];
      if (!std::isfinite(range)) {
        continue;
      }
      if (range < obstacle_range_min_ || range > obstacle_range_max_) {
        continue;
      }

      const double beam_angle =
          robot_yaw + latest_scan_.angle_min + static_cast<double>(i) * latest_scan_.angle_increment;
      const double wx = robot_x + static_cast<double>(range) * std::cos(beam_angle);
      const double wy = robot_y + static_cast<double>(range) * std::sin(beam_angle);

      int gx = 0;
      int gy = 0;
      if (!WorldToGrid(wx, wy, &gx, &gy)) {
        continue;
      }
      MarkOccupied(&merged_map, gx, gy);
    }

    merged_map_pub_.publish(merged_map);
    last_publish_time_ = now;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber map_sub_;
  ros::Subscriber localization_sub_;
  ros::Subscriber scan_sub_;
  ros::Publisher merged_map_pub_;

  nav_msgs::OccupancyGrid static_map_;
  nav_msgs::Odometry latest_localization_;
  sensor_msgs::LaserScan latest_scan_;
  bool have_static_map_ = false;
  bool have_localization_ = false;
  bool have_scan_ = false;
  ros::Time last_publish_time_;

  std::string static_map_topic_;
  std::string scan_topic_;
  std::string localization_topic_;
  std::string merged_map_topic_;
  double obstacle_range_min_ = 0.05;
  double obstacle_range_max_ = 8.0;
  double publish_rate_hz_ = 1.0;
  int obstacle_inflation_cells_ = 1;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "dynamic_obstacle_adapter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  DynamicObstacleAdapter node(nh, pnh);
  ros::spin();
  return 0;
}
