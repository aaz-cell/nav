#include <ros/ros.h>

#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>

#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl/filters/conditional_removal.h>         //条件滤波器头文件
#include <pcl/filters/passthrough.h>                 //直通滤波器头文件
#include <pcl/filters/radius_outlier_removal.h>      //半径滤波器头文件
#include <pcl/filters/statistical_outlier_removal.h> //统计滤波器头文件
#include <pcl/filters/voxel_grid.h>                  //体素滤波器头文件
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <string>
#include <vector>

std::string file_directory;
std::string file_name;
std::string pcd_file;

std::string map_topic_name;
std::string map_cloud_source = "auto";

const std::string pcd_format = ".pcd";

nav_msgs::OccupancyGrid map_topic_msg;
//最小和最大高度
double thre_z_min = 0.3;
double thre_z_max = 2.0;
int flag_pass_through = 0;
double map_resolution = 0.05;
double thre_radius = 0.1;
//半径滤波的点数阈值
int thres_point_count = 10;
bool enable_radius_filter = true;
double radius_filter_min_keep_ratio = 0.85;
int map_dilation_radius = 1;
double input_file_wait_timeout_sec = 10.0;
double input_file_poll_interval_sec = 0.2;

//直通滤波后数据指针
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);
//半径滤波后数据指针
pcl::PointCloud<pcl::PointXYZ>::Ptr
    cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr
    pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);

//直通滤波
void PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in);
//半径滤波
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud,
                         const double &radius, const int &thre_count);
//转换为栅格地图数据并发布
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::OccupancyGrid &msg);
const pcl::PointCloud<pcl::PointXYZ>::Ptr &SelectMapCloud();
void DilateOccupancyGrid(nav_msgs::OccupancyGrid &msg, int dilation_radius);
bool WaitForInputFile(const std::string &path, double timeout_sec,
                      double poll_interval_sec);

int main(int argc, char **argv) {
  ros::init(argc, argv, "pcl_filters");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Rate loop_rate(1.0);

  private_nh.param("file_directory", file_directory, std::string("/home/"));

  private_nh.param("file_name", file_name, std::string("map"));

  pcd_file = file_directory + file_name + pcd_format;

  private_nh.param("thre_z_min", thre_z_min, 0.2);
  private_nh.param("thre_z_max", thre_z_max, 2.0);
  private_nh.param("flag_pass_through", flag_pass_through, 0);
  private_nh.param("thre_radius", thre_radius, 0.5);
  private_nh.param("map_resolution", map_resolution, 0.05);
  private_nh.param("thres_point_count", thres_point_count, 10);
  private_nh.param("map_topic_name", map_topic_name, std::string("map"));
  private_nh.param("enable_radius_filter", enable_radius_filter, true);
  private_nh.param("radius_filter_min_keep_ratio", radius_filter_min_keep_ratio,
                   0.85);
  private_nh.param("map_cloud_source", map_cloud_source, std::string("auto"));
  private_nh.param("map_dilation_radius", map_dilation_radius, 1);
  private_nh.param("input_file_wait_timeout_sec", input_file_wait_timeout_sec,
                   10.0);
  private_nh.param("input_file_poll_interval_sec", input_file_poll_interval_sec,
                   0.2);

  ros::Publisher map_topic_pub =
      nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);

  if (!WaitForInputFile(pcd_file, input_file_wait_timeout_sec,
                        input_file_poll_interval_sec)) {
    ROS_ERROR("Timed out waiting for input file: %s", pcd_file.c_str());
    return -1;
  }

  // 下载pcd文件
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1) {
    PCL_ERROR("Couldn't read file: %s \n", pcd_file.c_str());
    return (-1);
  }

  std::cout << "初始点云数据点数：" << pcd_cloud->points.size() << std::endl;
  //对数据进行直通滤波
  PassThroughFilter(thre_z_min, thre_z_max, bool(flag_pass_through));
  //对数据进行半径滤波
  if (enable_radius_filter) {
    RadiusOutlierFilter(cloud_after_PassThrough, thre_radius, thres_point_count);
  } else {
    *cloud_after_Radius = *cloud_after_PassThrough;
    pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_radius_filter.pcd",
                                        *cloud_after_Radius);
    ROS_INFO("radius filter disabled, reuse pass-through cloud for debug output.");
  }
  //转换为栅格地图数据并发布
  SetMapTopicMsg(SelectMapCloud(), map_topic_msg);

  while (ros::ok()) {
    map_topic_pub.publish(map_topic_msg);

    loop_rate.sleep();

    ros::spinOnce();
  }

  return 0;
}

//直通滤波器对点云进行过滤，获取设定高度范围内的数据
void PassThroughFilter(const double &thre_low, const double &thre_high,
                       const bool &flag_in) {
  // 创建滤波器对象
  pcl::PassThrough<pcl::PointXYZ> passthrough;
  //输入点云
  passthrough.setInputCloud(pcd_cloud);
  //设置对z轴进行操作
  passthrough.setFilterFieldName("z");
  //设置滤波范围
  passthrough.setFilterLimits(thre_low, thre_high);
  // true表示保留滤波范围外，false表示保留范围内
  passthrough.setFilterLimitsNegative(flag_in);
  //执行滤波并存储
  passthrough.filter(*cloud_after_PassThrough);
  // test 保存滤波后的点云到文件
  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_filter.pcd",
                                      *cloud_after_PassThrough);
  std::cout << "直通滤波后点云数据点数："
            << cloud_after_PassThrough->points.size() << std::endl;
}

//半径滤波
void RadiusOutlierFilter(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pcd_cloud0,
                         const double &radius, const int &thre_count) {
  //创建滤波器
  pcl::RadiusOutlierRemoval<pcl::PointXYZ> radiusoutlier;
  //设置输入点云
  radiusoutlier.setInputCloud(pcd_cloud0);
  //设置半径,在该范围内找临近点
  radiusoutlier.setRadiusSearch(radius);
  //设置查询点的邻域点集数，小于该阈值的删除
  radiusoutlier.setMinNeighborsInRadius(thre_count);
  radiusoutlier.filter(*cloud_after_Radius);
  // test 保存滤波后的点云到文件
  pcl::io::savePCDFile<pcl::PointXYZ>(file_directory + "map_radius_filter.pcd",
                                      *cloud_after_Radius);
  std::cout << "半径滤波后点云数据点数：" << cloud_after_Radius->points.size()
            << std::endl;
}

const pcl::PointCloud<pcl::PointXYZ>::Ptr &SelectMapCloud() {
  if (map_cloud_source == "pass_through") {
    ROS_INFO("map_cloud_source=pass_through, use pass-through cloud for map.");
    return cloud_after_PassThrough;
  }

  if (map_cloud_source == "radius") {
    ROS_INFO("map_cloud_source=radius, use radius-filtered cloud for map.");
    return cloud_after_Radius;
  }

  const size_t pass_count = cloud_after_PassThrough->points.size();
  const size_t radius_count = cloud_after_Radius->points.size();
  const double keep_ratio =
      pass_count == 0 ? 0.0 : static_cast<double>(radius_count) / pass_count;

  if (!enable_radius_filter) {
    ROS_INFO("radius filter disabled, auto mode uses pass-through cloud.");
    return cloud_after_PassThrough;
  }

  if (radius_count == 0) {
    ROS_WARN("radius filter removed all points, fallback to pass-through cloud.");
    return cloud_after_PassThrough;
  }

  if (keep_ratio < radius_filter_min_keep_ratio) {
    ROS_WARN("radius filter keep ratio %.3f < %.3f, fallback to pass-through cloud.",
             keep_ratio, radius_filter_min_keep_ratio);
    return cloud_after_PassThrough;
  }

  ROS_INFO("radius filter keep ratio %.3f >= %.3f, use radius-filtered cloud.",
           keep_ratio, radius_filter_min_keep_ratio);
  return cloud_after_Radius;
}

bool WaitForInputFile(const std::string &path, double timeout_sec,
                      double poll_interval_sec) {
  const ros::Time start_time = ros::Time::now();
  const ros::Duration timeout(std::max(0.0, timeout_sec));
  const ros::Duration poll(std::max(0.05, poll_interval_sec));

  while (ros::ok()) {
    std::ifstream input(path.c_str(), std::ios::binary);
    if (input.good()) {
      return true;
    }

    if (timeout_sec <= 0.0) {
      return false;
    }

    if (ros::Time::now() - start_time >= timeout) {
      return false;
    }

    poll.sleep();
  }

  return false;
}

void DilateOccupancyGrid(nav_msgs::OccupancyGrid &msg, int dilation_radius) {
  if (dilation_radius <= 0 || msg.data.empty() || msg.info.width == 0 ||
      msg.info.height == 0) {
    return;
  }

  const int width = static_cast<int>(msg.info.width);
  const int height = static_cast<int>(msg.info.height);
  std::vector<int8_t> dilated = msg.data;

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const int index = x + y * width;
      if (msg.data[index] != 100) {
        continue;
      }

      for (int dy = -dilation_radius; dy <= dilation_radius; ++dy) {
        const int ny = y + dy;
        if (ny < 0 || ny >= height) {
          continue;
        }

        for (int dx = -dilation_radius; dx <= dilation_radius; ++dx) {
          const int nx = x + dx;
          if (nx < 0 || nx >= width) {
            continue;
          }

          if (dx * dx + dy * dy > dilation_radius * dilation_radius) {
            continue;
          }

          dilated[nx + ny * width] = 100;
        }
      }
    }
  }

  msg.data.swap(dilated);
}

//转换为栅格地图数据并发布
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                    nav_msgs::OccupancyGrid &msg) {
  msg.header.seq = 0;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.info.map_load_time = ros::Time::now();
  msg.info.resolution = map_resolution;

  double x_min = std::numeric_limits<double>::max();
  double x_max = std::numeric_limits<double>::lowest();
  double y_min = std::numeric_limits<double>::max();
  double y_max = std::numeric_limits<double>::lowest();

  if (cloud->points.empty()) {
    ROS_WARN("pcd is empty!\n");
    return;
  }

  for (size_t i = 0; i < cloud->points.size(); ++i) {
    const double x = cloud->points[i].x;
    const double y = cloud->points[i].y;

    x_min = std::min(x_min, x);
    x_max = std::max(x_max, x);
    y_min = std::min(y_min, y);
    y_max = std::max(y_max, y);
  }
  // origin的确定
  msg.info.origin.position.x = x_min;
  msg.info.origin.position.y = y_min;
  msg.info.origin.position.z = 0.0;
  msg.info.origin.orientation.x = 0.0;
  msg.info.origin.orientation.y = 0.0;
  msg.info.origin.orientation.z = 0.0;
  msg.info.origin.orientation.w = 1.0;
  //设置栅格地图大小
  msg.info.width =
      std::max(1, static_cast<int>(std::ceil((x_max - x_min) / map_resolution)) + 1);
  msg.info.height =
      std::max(1, static_cast<int>(std::ceil((y_max - y_min) / map_resolution)) + 1);
  //实际地图中某点坐标为(x,y)，对应栅格地图中坐标为[x*map.info.width+y]
  msg.data.resize(msg.info.width * msg.info.height);
  msg.data.assign(msg.info.width * msg.info.height, 0);

  ROS_INFO("data size = %zu\n", msg.data.size());

  for (size_t iter = 0; iter < cloud->points.size(); ++iter) {
    int i = int((cloud->points[iter].x - x_min) / map_resolution);
    if (i < 0 || i >= msg.info.width)
      continue;

    int j = int((cloud->points[iter].y - y_min) / map_resolution);
    if (j < 0 || j >= msg.info.height)
      continue;
    // 栅格地图的占有概率[0,100]，这里设置为占据
    msg.data[i + j * msg.info.width] = 100;
  }

  DilateOccupancyGrid(msg, map_dilation_radius);
}
