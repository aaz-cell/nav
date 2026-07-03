#include <ros/ros.h>
#include <ros/topic.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <tf2/utils.h>
#include "isweep_planner/GlobalReferencePoint.h"
#include "isweep_planner/RiskAwareGlobalReference.h"
#include "isweep_planner/local_planner/risk_aware_local_planner.h"
#include "isweep_planner/ros_interface/ros_visualizer.h"
#include <visualization_msgs/MarkerArray.h>

#include <Eigen/Core>

#include <algorithm>
#include <cmath>
#include <string>

#include "isweep_planner/framework/svsdf_runtime.h"

namespace isweep_planner {

class IsweepPlannerNode {
 public:
  IsweepPlannerNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
      : nh_(nh),
        pnh_(pnh),
        local_planner_(runtime_.grid_map(), runtime_.collision_checker(),
                       runtime_.footprint()) {
    LoadParams();
    runtime_.Initialize(nh_, pnh_);
    if (run_local_planner_) {
      local_planner_.Initialize(pnh_);
    }
    ROS_INFO("iSweep Planner coarse-to-fine runtime initialized.");
    WarmupInitialMap();
    SetupPubSub();
    TryPlan();
  }

 private:
  void LoadParams() {
    pnh_.param("use_start_goal_params", use_start_goal_params_, true);
    pnh_.param("startup_wait_for_map", startup_wait_for_map_, true);
    pnh_.param("startup_map_wait_timeout", startup_map_wait_timeout_sec_, 0.0);
    pnh_.param("startup_map_wait_poll", startup_map_wait_poll_sec_, 1.0);
    pnh_.param("replan_on_map_update", replan_on_map_update_, false);
    pnh_.param("run_local_planner", run_local_planner_, true);
    if (startup_map_wait_poll_sec_ < 0.1) {
      startup_map_wait_poll_sec_ = 0.1;
    }

    start_ = LoadPoseParam("start_pose");
    goal_ = LoadPoseParam("goal_pose");
    start_ready_ = use_start_goal_params_;
    goal_ready_ = use_start_goal_params_;
    if (use_start_goal_params_) {
      ROS_INFO("Using parameter start/goal: start=(%.2f, %.2f, %.2f), goal=(%.2f, %.2f, %.2f)",
               start_.x(), start_.y(), start_.z(), goal_.x(), goal_.y(), goal_.z());
      current_state_.x = start_.x();
      current_state_.y = start_.y();
      current_state_.yaw = start_.z();
      current_pose_ready_ = true;
    }

    pnh_.param("local_planner/update_rate", local_planner_update_rate_hz_, 10.0);
    if (local_planner_update_rate_hz_ < 0.5) {
      local_planner_update_rate_hz_ = 0.5;
    }
    pnh_.param("local_planner/cmd_linear_sign", cmd_linear_sign_, 1.0);
    pnh_.param("local_planner/cmd_angular_sign", cmd_angular_sign_, 1.0);
    cmd_linear_sign_ = cmd_linear_sign_ >= 0.0 ? 1.0 : -1.0;
    cmd_angular_sign_ = cmd_angular_sign_ >= 0.0 ? 1.0 : -1.0;
  }

  Eigen::Vector3d LoadPoseParam(const std::string& key) {
    Eigen::Vector3d pose = Eigen::Vector3d::Zero();
    pnh_.param(key + "/x", pose.x(), 0.0);
    pnh_.param(key + "/y", pose.y(), 0.0);
    pnh_.param(key + "/yaw", pose.z(), 0.0);
    return pose;
  }

  bool SameMap(const nav_msgs::OccupancyGrid& lhs,
               const nav_msgs::OccupancyGrid& rhs) const {
    return lhs.header.frame_id == rhs.header.frame_id &&
           lhs.info.width == rhs.info.width &&
           lhs.info.height == rhs.info.height &&
           std::abs(lhs.info.resolution - rhs.info.resolution) < 1e-9 &&
           std::abs(lhs.info.origin.position.x - rhs.info.origin.position.x) < 1e-9 &&
           std::abs(lhs.info.origin.position.y - rhs.info.origin.position.y) < 1e-9 &&
           std::abs(lhs.info.origin.position.z - rhs.info.origin.position.z) < 1e-9 &&
           std::abs(lhs.info.origin.orientation.x - rhs.info.origin.orientation.x) < 1e-9 &&
           std::abs(lhs.info.origin.orientation.y - rhs.info.origin.orientation.y) < 1e-9 &&
           std::abs(lhs.info.origin.orientation.z - rhs.info.origin.orientation.z) < 1e-9 &&
           std::abs(lhs.info.origin.orientation.w - rhs.info.origin.orientation.w) < 1e-9 &&
           lhs.data == rhs.data;
  }

  void WarmupInitialMap() {
    if (!startup_wait_for_map_) {
      ROS_INFO("Startup warmup: disabled; planner will wait for /map through subscriber callbacks.");
      return;
    }

    if (startup_map_wait_timeout_sec_ > 0.0) {
      ROS_INFO("Startup warmup: waiting up to %.1fs for the first /map before enabling planning.",
               startup_map_wait_timeout_sec_);
    } else {
      ROS_INFO("Startup warmup: waiting for the first /map before enabling planning.");
    }

    const ros::WallTime wait_start = ros::WallTime::now();
    while (ros::ok()) {
      const double elapsed = (ros::WallTime::now() - wait_start).toSec();
      double poll_timeout_sec = startup_map_wait_poll_sec_;
      if (startup_map_wait_timeout_sec_ > 0.0) {
        const double remaining = startup_map_wait_timeout_sec_ - elapsed;
        if (remaining <= 1e-6) {
          ROS_WARN("Startup warmup: timed out after %.3fs waiting for /map; falling back to normal subscriber-driven map updates.",
                   elapsed);
          return;
        }
        if (remaining < poll_timeout_sec) {
          poll_timeout_sec = remaining;
        }
      }

      nav_msgs::OccupancyGrid::ConstPtr msg =
          ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(
              "/map", nh_, ros::Duration(poll_timeout_sec));
      if (!msg) {
        ROS_INFO("Startup warmup: still waiting for /map after %.1fs",
                 (ros::WallTime::now() - wait_start).toSec());
        continue;
      }

      const double wait_time = (ros::WallTime::now() - wait_start).toSec();
      ROS_INFO("Startup warmup: received initial /map after %.3fs (%u x %u @ %.3f m); updating runtime and warming topology cache.",
               wait_time, msg->info.width, msg->info.height, msg->info.resolution);

      const ros::WallTime update_start = ros::WallTime::now();
      if (!runtime_.UpdateMap(*msg)) {
        ROS_ERROR("Startup warmup: failed to update planner state from the initial /map; continuing to wait for another map message.");
        continue;
      }

      map_ready_ = true;
      startup_map_ = *msg;
      skip_duplicate_startup_map_ = true;
      const double update_time = (ros::WallTime::now() - update_start).toSec();
      ROS_INFO("Startup warmup: initial /map integrated in %.3fs. Map preload complete; planner is ready%s",
               update_time,
               use_start_goal_params_ ? " to plan with parameter start/goal."
                                      : " for RViz start/goal input.");
      return;
    }

    ROS_WARN("Startup warmup: ROS shutdown requested before the first /map arrived.");
  }

  void SetupPubSub() {
    map_sub_ = nh_.subscribe("/map", 1, &IsweepPlannerNode::MapCallback, this);
    start_sub_ = nh_.subscribe("/initialpose", 1, &IsweepPlannerNode::StartCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &IsweepPlannerNode::GoalCallback, this);
    trajectory_pub_ = nh_.advertise<nav_msgs::Path>("/isweep_planner/trajectory", 1, true);
    coarse_path_pub_ = nh_.advertise<nav_msgs::Path>("/isweep_planner/trajectory_astar", 1, true);
    risk_aware_reference_pub_ =
        nh_.advertise<isweep_planner::RiskAwareGlobalReference>(
            "/isweep_planner/risk_aware_global_reference", 1, true);
    reference_risk_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("/isweep_planner/reference_risk", 1, true);
    reference_clearance_pub_ =
        nh_.advertise<std_msgs::Float64MultiArray>("/isweep_planner/reference_clearance", 1, true);
    time_pub_ = nh_.advertise<std_msgs::Float64>("/isweep_planner/planning_time", 1, true);
    stats_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/isweep_planner/planning_stats", 1, true);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/isweep_planner/markers", 1, true);
    if (run_local_planner_) {
      current_pose_sub_ = nh_.subscribe("/isweep_planner/current_pose", 1,
                                        &IsweepPlannerNode::CurrentPoseCallback, this);
      current_velocity_sub_ = nh_.subscribe("/isweep_planner/current_velocity", 1,
                                            &IsweepPlannerNode::CurrentVelocityCallback, this);
      local_trajectory_pub_ =
          nh_.advertise<nav_msgs::Path>("/isweep_planner/local_trajectory", 1, true);
      local_cmd_pub_ =
          nh_.advertise<geometry_msgs::Twist>("/isweep_planner/local_cmd", 1, true);
      local_status_pub_ =
          nh_.advertise<std_msgs::String>("/isweep_planner/local_planner_status", 1, true);
      replan_request_pub_ =
          nh_.advertise<std_msgs::Bool>("/isweep_planner/replan_request", 1, true);
      local_marker_pub_ =
          nh_.advertise<visualization_msgs::MarkerArray>("/isweep_planner/local_markers", 1, true);
      local_planner_timer_ =
          nh_.createTimer(ros::Duration(1.0 / local_planner_update_rate_hz_),
                          &IsweepPlannerNode::LocalPlannerTimerCallback, this);
      ROS_INFO("iSweep local planner enabled at %.1f Hz.", local_planner_update_rate_hz_);
    } else {
      ROS_INFO("iSweep local planner disabled; global planning outputs only.");
    }

    if (!use_start_goal_params_) {
      ROS_INFO("Waiting for RViz /initialpose and /move_base_simple/goal.");
    }
  }

  void MapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    if (skip_duplicate_startup_map_ && SameMap(*msg, startup_map_)) {
      skip_duplicate_startup_map_ = false;
      ROS_INFO("Map callback: skipping duplicate startup /map already preloaded during warmup.");
      return;
    }
    skip_duplicate_startup_map_ = false;

    if (!runtime_.UpdateMap(*msg)) {
      ROS_ERROR("Failed to update planner map.");
      return;
    }
    map_ready_ = true;
    ROS_INFO("Map received: %u x %u @ %.3f m", msg->info.width, msg->info.height, msg->info.resolution);
    if (replan_on_map_update_ || !has_planned_once_) {
      TryPlan();
    } else {
      ROS_DEBUG("Map update integrated without global replan.");
    }
  }

  void StartCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
    start_.x() = msg->pose.pose.position.x;
    start_.y() = msg->pose.pose.position.y;
    start_.z() = tf2::getYaw(msg->pose.pose.orientation);
    start_ready_ = true;
    use_start_goal_params_ = false;
    current_state_.x = start_.x();
    current_state_.y = start_.y();
    current_state_.yaw = start_.z();
    current_pose_ready_ = true;
    has_planned_once_ = false;
    ROS_INFO("Start: (%.2f, %.2f, %.2f)", start_.x(), start_.y(), start_.z());
    TryPlan();
  }

  void CurrentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_state_.x = msg->pose.position.x;
    current_state_.y = msg->pose.position.y;
    current_state_.yaw = tf2::getYaw(msg->pose.orientation);
    current_pose_ready_ = true;
  }

  void CurrentVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    current_state_.v = msg->twist.linear.x;
    current_state_.has_velocity = true;
  }

  void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    goal_.x() = msg->pose.position.x;
    goal_.y() = msg->pose.position.y;
    goal_.z() = tf2::getYaw(msg->pose.orientation);
    goal_ready_ = true;
    use_start_goal_params_ = false;
    has_planned_once_ = false;
    ROS_INFO("Goal: (%.2f, %.2f, %.2f)", goal_.x(), goal_.y(), goal_.z());
    TryPlan();
  }

  void TryPlan() {
    if (!map_ready_ || !start_ready_ || !goal_ready_ || planning_) {
      return;
    }
    planning_ = true;
    RunPlanning();
    planning_ = false;
  }

  void RunPlanning() {
    ROS_INFO("=== iSweep Planner: Starting coarse-to-fine planning ===");
    const SvsdfPlanResult result = runtime_.Plan(start_, goal_);

    coarse_path_pub_.publish(result.coarse_path);
    trajectory_pub_.publish(result.trajectory);
    latest_global_reference_ = result.risk_aware_global_reference;
    PublishRiskAwareReference(result.risk_aware_global_reference);
    PublishStats(result.stats);
    PublishPlanningTime(result.stats.total_solve_time);
    
    if (!result.raw_trajectory.empty() ||
        !result.risk_aware_global_reference.points.empty()) {
      visualization_msgs::MarkerArray markers =
          BuildPlannerMarkers(result.raw_trajectory, result.risk_aware_global_reference);
      marker_pub_.publish(markers);
    } else {
      PublishEmptyMarkers();
    }

    if (result.success) {
      has_planned_once_ = true;
      ROS_INFO("Planning done: %.3f s, coarse=%d support=%d local_obs=%d min_clearance=%.3f",
               result.stats.total_solve_time,
               result.stats.coarse_path_points,
               result.stats.support_points,
               result.stats.local_obstacle_points,
               result.stats.min_clearance);
    } else {
      ROS_WARN("Planning failed after %.3f s (coarse=%d support=%d local_obs=%d min_clearance=%.3f)",
               result.stats.total_solve_time,
               result.stats.coarse_path_points,
               result.stats.support_points,
               result.stats.local_obstacle_points,
               result.stats.min_clearance);
    }
  }

  void PublishPlanningTime(double seconds) {
    std_msgs::Float64 msg;
    msg.data = seconds;
    time_pub_.publish(msg);
  }

  void PublishStats(const PlanningStats& stats) {
    std_msgs::Float64MultiArray msg;
    msg.data = stats.asVector();
    stats_pub_.publish(msg);
  }

  void PublishEmptyMarkers() {
    visualization_msgs::MarkerArray msg;
    visualization_msgs::Marker marker;
    marker.action = visualization_msgs::Marker::DELETEALL;
    msg.markers.push_back(marker);
    marker_pub_.publish(msg);
  }

  isweep_planner::RiskAwareGlobalReference ToRosMessage(
      const RiskAwareGlobalReferenceData& reference) const {
    isweep_planner::RiskAwareGlobalReference msg;
    msg.header.stamp = reference.stamp;
    msg.header.frame_id = reference.frame_id;
    msg.total_length = reference.total_length;
    msg.num_segments = std::max(0, reference.num_segments);
    msg.num_high_risk_segments = std::max(0, reference.num_high_risk_segments);
    msg.points.reserve(reference.points.size());
    for (size_t i = 0; i < reference.points.size(); ++i) {
      const GlobalReferencePointData& point = reference.points[i];
      isweep_planner::GlobalReferencePoint ros_point;
      ros_point.x = point.x;
      ros_point.y = point.y;
      ros_point.yaw = point.yaw;
      ros_point.risk_level = static_cast<uint8_t>(std::max(0, point.risk_level));
      ros_point.clearance = point.clearance;
      ros_point.segment_id = std::max(0, point.segment_id);
      ros_point.s = point.s;
      ros_point.preferred_speed = point.preferred_speed;
      msg.points.push_back(ros_point);
    }
    return msg;
  }

  void PublishRiskAwareReference(const RiskAwareGlobalReferenceData& reference) {
    risk_aware_reference_pub_.publish(ToRosMessage(reference));

    std_msgs::Float64MultiArray risk_msg;
    std_msgs::Float64MultiArray clearance_msg;
    risk_msg.data.reserve(reference.points.size());
    clearance_msg.data.reserve(reference.points.size());
    for (size_t i = 0; i < reference.points.size(); ++i) {
      risk_msg.data.push_back(static_cast<double>(reference.points[i].risk_level));
      clearance_msg.data.push_back(reference.points[i].clearance);
    }
    reference_risk_pub_.publish(risk_msg);
    reference_clearance_pub_.publish(clearance_msg);
  }

  void LocalPlannerTimerCallback(const ros::TimerEvent&) {
    if (!runtime_.map_ready() || !current_pose_ready_ ||
        latest_global_reference_.points.empty()) {
      return;
    }

    const LocalPlanningResult result =
        local_planner_.Plan(current_state_, latest_global_reference_);

    nav_msgs::Path local_path;
    if (!result.local_path.empty()) {
      local_path = RosVisualizer::MakePath(result.local_path);
    }
    local_path.header.stamp = ros::Time::now();
    local_path.header.frame_id =
        latest_global_reference_.frame_id.empty() ? "map"
                                                  : latest_global_reference_.frame_id;
    local_trajectory_pub_.publish(local_path);

    geometry_msgs::Twist cmd_msg;
    cmd_msg.linear.x = cmd_linear_sign_ * result.local_cmd.linear_velocity;
    cmd_msg.angular.z = cmd_angular_sign_ * result.local_cmd.angular_velocity;
    local_cmd_pub_.publish(cmd_msg);

    std_msgs::String status_msg;
    status_msg.data = std::string(ToString(result.status)) + "|" +
                      ToString(result.mode) + "|" + result.debug_reason;
    local_status_pub_.publish(status_msg);

    std_msgs::Bool replan_msg;
    replan_msg.data = result.need_replan;
    replan_request_pub_.publish(replan_msg);

    local_marker_pub_.publish(RosVisualizer::MakeLocalPlannerMarkers(
        result, latest_global_reference_.frame_id.empty()
                    ? "map"
                    : latest_global_reference_.frame_id));
  }

  visualization_msgs::MarkerArray BuildPlannerMarkers(
      const Trajectory& trajectory,
      const RiskAwareGlobalReferenceData& reference) const {
    const std::string frame_id = reference.frame_id.empty() ? "map" : reference.frame_id;
    visualization_msgs::MarkerArray markers =
        RosVisualizer::MakeRiskAwareReferenceMarkers(reference, frame_id);
    if (trajectory.empty()) {
      return markers;
    }

    visualization_msgs::MarkerArray footprint_markers =
        RosVisualizer::MakeFootprintMarkers(trajectory, runtime_.footprint(), frame_id, 0.5);
    if (!footprint_markers.markers.empty()) {
      footprint_markers.markers.erase(footprint_markers.markers.begin());
    }
    markers.markers.insert(markers.markers.end(), footprint_markers.markers.begin(),
                           footprint_markers.markers.end());
    return markers;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber map_sub_;
  ros::Subscriber start_sub_;
  ros::Subscriber goal_sub_;
  ros::Subscriber current_pose_sub_;
  ros::Subscriber current_velocity_sub_;
  ros::Publisher trajectory_pub_;
  ros::Publisher coarse_path_pub_;
  ros::Publisher risk_aware_reference_pub_;
  ros::Publisher reference_risk_pub_;
  ros::Publisher reference_clearance_pub_;
  ros::Publisher time_pub_;
  ros::Publisher stats_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher local_trajectory_pub_;
  ros::Publisher local_cmd_pub_;
  ros::Publisher local_status_pub_;
  ros::Publisher replan_request_pub_;
  ros::Publisher local_marker_pub_;
  ros::Timer local_planner_timer_;

  SvsdfRuntime runtime_;
  RiskAwareLocalPlanner local_planner_;
  RiskAwareGlobalReferenceData latest_global_reference_;
  LocalPlannerState current_state_;
  Eigen::Vector3d start_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
  bool use_start_goal_params_ = true;
  bool startup_wait_for_map_ = true;
  double startup_map_wait_timeout_sec_ = 0.0;
  double startup_map_wait_poll_sec_ = 1.0;
  bool skip_duplicate_startup_map_ = false;
  nav_msgs::OccupancyGrid startup_map_;
  bool map_ready_ = false;
  bool start_ready_ = false;
  bool goal_ready_ = false;
  bool current_pose_ready_ = false;
  bool planning_ = false;
  bool replan_on_map_update_ = false;
  bool run_local_planner_ = true;
  bool has_planned_once_ = false;
  double local_planner_update_rate_hz_ = 10.0;
  double cmd_linear_sign_ = 1.0;
  double cmd_angular_sign_ = 1.0;
};

}  // namespace isweep_planner

int main(int argc, char** argv) {
  ros::init(argc, argv, "isweep_planner");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  isweep_planner::IsweepPlannerNode node(nh, pnh);
  ros::spin();
  return 0;
}
