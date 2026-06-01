#include "isweep_planner/ros_interface/ros_visualizer.h"
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>

namespace isweep_planner {

nav_msgs::Path RosVisualizer::MakePath(const std::vector<SE2State>& states) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  for (size_t i = 0; i < states.size(); ++i) {
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = states[i].x;
    pose.pose.position.y = states[i].y;
    pose.pose.orientation.z = std::sin(states[i].yaw * 0.5);
    pose.pose.orientation.w = std::cos(states[i].yaw * 0.5);
    path.poses.push_back(pose);
  }
  return path;
}

nav_msgs::Path RosVisualizer::MakePathFromTopo(const TopoPath& path_in) {
  std::vector<SE2State> states;
  states.reserve(path_in.waypoints.size());
  for (size_t i = 0; i < path_in.waypoints.size(); ++i) {
    const TopoWaypoint& wp = path_in.waypoints[i];
    states.push_back(SE2State(wp.pos.x(), wp.pos.y(), wp.yaw));
  }
  return MakePath(states);
}

nav_msgs::Path RosVisualizer::MakeTrajectoryPath(const Trajectory& traj) {
  nav_msgs::Path path;
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "map";
  if (traj.empty()) {
    ROS_WARN("MakeTrajectoryPath: traj is empty!");
    return path;
  }

  const double total = traj.totalDuration();
  ROS_INFO("MakeTrajectoryPath: pieces=%zu total_duration=%.4f", traj.pos_pieces.size(), total);
  if (!traj.pos_pieces.empty()) {
    SE2State s0 = traj.sample(0.0);
    SE2State sf = traj.sample(std::max(0.0, total - 0.01));
    ROS_INFO("MakeTrajectoryPath: start=(%.3f,%.3f,%.3f) end=(%.3f,%.3f,%.3f)",
             s0.x, s0.y, s0.yaw, sf.x, sf.y, sf.yaw);
  }

  const double min_sample_dt = 0.05;
  const double max_sample_dt = 0.40;
  const size_t min_pose_count = 1000;
  const size_t max_pose_count = 2500;
  const size_t target_pose_count =
      std::max(min_pose_count,
               std::min(max_pose_count,
                        traj.pos_pieces.size() * static_cast<size_t>(2)));
  const double sample_dt =
      total > 1e-9
          ? std::min(max_sample_dt,
                     std::max(min_sample_dt,
                              total / static_cast<double>(target_pose_count)))
          : min_sample_dt;
  const size_t estimated_pose_count =
      total > 1e-9 ? static_cast<size_t>(std::ceil(total / sample_dt)) + 1 : 1;
  path.poses.reserve(estimated_pose_count);
  ROS_INFO("MakeTrajectoryPath: sample_dt=%.3f target_poses=%zu estimated_poses=%zu",
           sample_dt, target_pose_count, estimated_pose_count);

  for (double t = 0.0; t < total; t += sample_dt) {
    const SE2State state = traj.sample(t);
    geometry_msgs::PoseStamped pose;
    pose.header = path.header;
    pose.pose.position.x = state.x;
    pose.pose.position.y = state.y;
    pose.pose.orientation.z = std::sin(state.yaw * 0.5);
    pose.pose.orientation.w = std::cos(state.yaw * 0.5);
    path.poses.push_back(pose);
  }

  const SE2State final_state = traj.sample(total);
  geometry_msgs::PoseStamped final_pose;
  final_pose.header = path.header;
  final_pose.pose.position.x = final_state.x;
  final_pose.pose.position.y = final_state.y;
  final_pose.pose.orientation.z = std::sin(final_state.yaw * 0.5);
  final_pose.pose.orientation.w = std::cos(final_state.yaw * 0.5);
  path.poses.push_back(final_pose);
  return path;
}

visualization_msgs::MarkerArray RosVisualizer::MakeFootprintMarkers(
    const Trajectory& traj,
    const FootprintModel& footprint,
    const std::string& frame_id,
    double dt) {
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  msg.markers.push_back(delete_marker);

  if (traj.empty()) {
    return msg;
  }

  const double total = traj.totalDuration();
  int marker_id = 0;

  for (double t = 0.0; t <= total + 1e-9; t += dt) {
    const SE2State state = traj.sample(std::min(t, total));

    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "footprint";
    marker.id = marker_id++;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.scale.x = 0.02;
    marker.color.a = 0.6;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    const std::vector<Eigen::Vector2d>& local_poly = footprint.vertices();
    for (const Eigen::Vector2d& pt : local_poly) {
      double wx = state.x + pt.x() * std::cos(state.yaw) - pt.y() * std::sin(state.yaw);
      double wy = state.y + pt.x() * std::sin(state.yaw) + pt.y() * std::cos(state.yaw);
      geometry_msgs::Point p;
      p.x = wx;
      p.y = wy;
      p.z = 0.0;
      marker.points.push_back(p);
    }
    if (!local_poly.empty()) {
      const Eigen::Vector2d& pt = local_poly.front();
      double wx = state.x + pt.x() * std::cos(state.yaw) - pt.y() * std::sin(state.yaw);
      double wy = state.y + pt.x() * std::sin(state.yaw) + pt.y() * std::cos(state.yaw);
      geometry_msgs::Point p;
      p.x = wx;
      p.y = wy;
      p.z = 0.0;
      marker.points.push_back(p);
    }

    msg.markers.push_back(marker);
  }

  return msg;
}

visualization_msgs::MarkerArray RosVisualizer::MakeRiskAwareReferenceMarkers(
    const RiskAwareGlobalReferenceData& reference, const std::string& frame_id) {
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  msg.markers.push_back(delete_marker);

  if (reference.points.empty()) {
    return msg;
  }

  visualization_msgs::Marker low_marker;
  low_marker.header.frame_id = frame_id;
  low_marker.header.stamp = reference.stamp;
  low_marker.ns = "risk_reference_low";
  low_marker.id = 0;
  low_marker.type = visualization_msgs::Marker::LINE_LIST;
  low_marker.action = visualization_msgs::Marker::ADD;
  low_marker.scale.x = 0.06;
  low_marker.color.a = 0.9;
  low_marker.color.r = 0.1;
  low_marker.color.g = 0.8;
  low_marker.color.b = 0.2;

  visualization_msgs::Marker high_marker = low_marker;
  high_marker.ns = "risk_reference_high";
  high_marker.id = 1;
  high_marker.color.r = 0.95;
  high_marker.color.g = 0.2;
  high_marker.color.b = 0.15;

  visualization_msgs::Marker low_points;
  low_points.header.frame_id = frame_id;
  low_points.header.stamp = reference.stamp;
  low_points.ns = "risk_reference_low_points";
  low_points.id = 2;
  low_points.type = visualization_msgs::Marker::SPHERE_LIST;
  low_points.action = visualization_msgs::Marker::ADD;
  low_points.scale.x = 0.10;
  low_points.scale.y = 0.10;
  low_points.scale.z = 0.10;
  low_points.color.a = 0.95;
  low_points.color.r = 0.1;
  low_points.color.g = 0.8;
  low_points.color.b = 0.2;

  visualization_msgs::Marker high_points = low_points;
  high_points.ns = "risk_reference_high_points";
  high_points.id = 3;
  high_points.color.r = 0.95;
  high_points.color.g = 0.2;
  high_points.color.b = 0.15;

  for (size_t i = 0; i < reference.points.size(); ++i) {
    const GlobalReferencePointData& point = reference.points[i];
    geometry_msgs::Point p;
    p.x = point.x;
    p.y = point.y;
    p.z = 0.08;
    if (point.risk_level == 1) {
      high_points.points.push_back(p);
    } else {
      low_points.points.push_back(p);
    }
  }

  for (size_t i = 1; i < reference.points.size(); ++i) {
    const GlobalReferencePointData& prev = reference.points[i - 1];
    const GlobalReferencePointData& curr = reference.points[i];
    visualization_msgs::Marker& active_marker =
        curr.risk_level == 1 ? high_marker : low_marker;

    geometry_msgs::Point p0;
    p0.x = prev.x;
    p0.y = prev.y;
    p0.z = 0.05;

    geometry_msgs::Point p1;
    p1.x = curr.x;
    p1.y = curr.y;
    p1.z = 0.05;

    active_marker.points.push_back(p0);
    active_marker.points.push_back(p1);
  }

  msg.markers.push_back(low_marker);
  msg.markers.push_back(high_marker);
  msg.markers.push_back(low_points);
  msg.markers.push_back(high_points);
  return msg;
}

visualization_msgs::MarkerArray RosVisualizer::MakeLocalPlannerMarkers(
    const LocalPlanningResult& result, const std::string& frame_id) {
  visualization_msgs::MarkerArray msg;

  visualization_msgs::Marker delete_marker;
  delete_marker.action = visualization_msgs::Marker::DELETEALL;
  msg.markers.push_back(delete_marker);

  if (result.local_path.empty()) {
    return msg;
  }

  visualization_msgs::Marker path_marker;
  path_marker.header.frame_id = frame_id;
  path_marker.header.stamp = ros::Time::now();
  path_marker.ns = "local_plan";
  path_marker.id = 0;
  path_marker.type = visualization_msgs::Marker::LINE_STRIP;
  path_marker.action = visualization_msgs::Marker::ADD;
  path_marker.scale.x = 0.08;
  path_marker.color.a = 0.95;
  if (result.need_replan) {
    path_marker.color.r = 0.95;
    path_marker.color.g = 0.15;
    path_marker.color.b = 0.10;
  } else if (result.mode == LocalPlannerMode::HIGH_RISK_STRICT) {
    path_marker.color.r = 0.95;
    path_marker.color.g = 0.55;
    path_marker.color.b = 0.10;
  } else {
    path_marker.color.r = 0.10;
    path_marker.color.g = 0.55;
    path_marker.color.b = 0.95;
  }

  for (size_t i = 0; i < result.local_path.size(); ++i) {
    geometry_msgs::Point point;
    point.x = result.local_path[i].x;
    point.y = result.local_path[i].y;
    point.z = 0.08;
    path_marker.points.push_back(point);
  }
  msg.markers.push_back(path_marker);

  visualization_msgs::Marker ref_marker = path_marker;
  ref_marker.ns = "local_reference_window";
  ref_marker.id = 1;
  ref_marker.scale.x = 0.03;
  ref_marker.color.r = 1.0;
  ref_marker.color.g = 1.0;
  ref_marker.color.b = 1.0;
  ref_marker.color.a = 0.55;
  ref_marker.points.clear();
  for (size_t i = 0; i < result.local_reference.size(); ++i) {
    geometry_msgs::Point point;
    point.x = result.local_reference[i].x;
    point.y = result.local_reference[i].y;
    point.z = 0.03;
    ref_marker.points.push_back(point);
  }
  msg.markers.push_back(ref_marker);
  return msg;
}

}  // namespace isweep_planner
