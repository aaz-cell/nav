#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

namespace {

class NavigationAdapter {
 public:
  NavigationAdapter(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    LoadParams();

    map_sub_ = nh_.subscribe(map_in_topic_, 1, &NavigationAdapter::MapCallback, this);
    localization_sub_ =
        nh_.subscribe(localization_topic_, 10, &NavigationAdapter::LocalizationCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 20, &NavigationAdapter::OdomCallback, this);
    goal_sub_ = nh_.subscribe(goal_in_topic_, 10, &NavigationAdapter::GoalCallback, this);
    replan_sub_ = nh_.subscribe(replan_topic_, 10, &NavigationAdapter::ReplanCallback, this);

    map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>(map_out_topic_, 1, true);
    pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(current_pose_topic_, 10, true);
    velocity_pub_ =
        nh_.advertise<geometry_msgs::TwistStamped>(current_velocity_topic_, 10, true);
    planner_start_pub_ =
        nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(planner_start_topic_, 10, true);
    planner_goal_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(planner_goal_topic_, 10, true);

    ROS_INFO("navigation_adapter ready: %s -> %s, %s -> %s, %s -> %s/%s",
             map_in_topic_.c_str(), map_out_topic_.c_str(), localization_topic_.c_str(),
             current_pose_topic_.c_str(), goal_in_topic_.c_str(), planner_start_topic_.c_str(),
             planner_goal_topic_.c_str());
  }

 private:
  void LoadParams() {
    pnh_.param<std::string>("map_in_topic", map_in_topic_, "/prior_map");
    pnh_.param<std::string>("map_out_topic", map_out_topic_, "/isweep_planner/map");
    pnh_.param<std::string>("localization_topic", localization_topic_, "/localization");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/Odometry");
    pnh_.param<std::string>("goal_in_topic", goal_in_topic_, "/move_base_simple/goal");
    pnh_.param<std::string>("planner_goal_topic", planner_goal_topic_, "/isweep_planner/goal");
    pnh_.param<std::string>("planner_start_topic", planner_start_topic_,
                            "/isweep_planner/initialpose");
    pnh_.param<std::string>("current_pose_topic", current_pose_topic_,
                            "/isweep_planner/current_pose");
    pnh_.param<std::string>("current_velocity_topic", current_velocity_topic_,
                            "/isweep_planner/current_velocity");
    pnh_.param<std::string>("replan_topic", replan_topic_, "/isweep_planner/replan_request");
    pnh_.param<bool>("auto_start_from_localization", auto_start_from_localization_, true);
    pnh_.param<bool>("replan_on_request", replan_on_request_, true);
    pnh_.param<double>("goal_republish_delay", goal_republish_delay_sec_, 0.05);
    pnh_.param<double>("replan_request_min_interval", replan_request_min_interval_sec_, 1.0);
  }

  void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    nav_msgs::OccupancyGrid out = *msg;
    if (out.header.frame_id.empty()) {
      out.header.frame_id = "map";
    }
    map_pub_.publish(out);
  }

  void LocalizationCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    latest_localization_ = *msg;
    have_localization_ = true;

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header = msg->header;
    if (pose_msg.header.frame_id.empty()) {
      pose_msg.header.frame_id = "map";
    }
    pose_msg.pose = msg->pose.pose;
    pose_pub_.publish(pose_msg);
  }

  void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::TwistStamped velocity_msg;
    velocity_msg.header = msg->header;
    velocity_msg.twist = msg->twist.twist;
    velocity_pub_.publish(velocity_msg);
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    latest_goal_ = *msg;
    have_goal_ = true;
    PublishPlannerInputs("new_goal");
  }

  void ReplanCallback(const std_msgs::Bool::ConstPtr& msg) {
    const bool rising_edge = msg->data && !last_replan_request_;
    last_replan_request_ = msg->data;
    if (!replan_on_request_ || !rising_edge || !have_goal_) {
      return;
    }
    const ros::Time now = ros::Time::now();
    if (!last_replan_publish_time_.isZero() &&
        (now - last_replan_publish_time_).toSec() <
            replan_request_min_interval_sec_) {
      return;
    }
    last_replan_publish_time_ = now;
    PublishPlannerInputs("replan_request");
  }

  void PublishPlannerInputs(const char* reason) {
    if (!have_goal_) {
      return;
    }

    if (auto_start_from_localization_ && have_localization_) {
      geometry_msgs::PoseWithCovarianceStamped start_msg;
      start_msg.header = latest_localization_.header;
      if (start_msg.header.frame_id.empty()) {
        start_msg.header.frame_id = "map";
      }
      start_msg.pose.pose = latest_localization_.pose.pose;
      start_msg.pose.covariance = latest_localization_.pose.covariance;
      planner_start_pub_.publish(start_msg);
    }

    if (goal_republish_delay_sec_ > 1e-4) {
      ros::Duration(goal_republish_delay_sec_).sleep();
    }

    geometry_msgs::PoseStamped goal_msg = latest_goal_;
    if (goal_msg.header.frame_id.empty()) {
      goal_msg.header.frame_id = "map";
    }
    planner_goal_pub_.publish(goal_msg);
    ROS_INFO("navigation_adapter published planner start/goal (%s)", reason);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber map_sub_;
  ros::Subscriber localization_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber replan_sub_;
  ros::Publisher map_pub_;
  ros::Publisher pose_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher planner_start_pub_;
  ros::Publisher planner_goal_pub_;

  nav_msgs::Odometry latest_localization_;
  geometry_msgs::PoseStamped latest_goal_;
  bool have_localization_ = false;
  bool have_goal_ = false;

  std::string map_in_topic_;
  std::string map_out_topic_;
  std::string localization_topic_;
  std::string odom_topic_;
  std::string goal_in_topic_;
  std::string planner_goal_topic_;
  std::string planner_start_topic_;
  std::string current_pose_topic_;
  std::string current_velocity_topic_;
  std::string replan_topic_;
  bool auto_start_from_localization_ = true;
  bool replan_on_request_ = true;
  double goal_republish_delay_sec_ = 0.05;
  double replan_request_min_interval_sec_ = 1.0;
  bool last_replan_request_ = false;
  ros::Time last_replan_publish_time_;
};

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "navigation_adapter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  NavigationAdapter adapter(nh, pnh);
  ros::spin();
  return 0;
}
