#include "isweep_planner/local_planner/risk_aware_local_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace isweep_planner {

namespace {

double Distance2d(double x0, double y0, double x1, double y1) {
  const double dx = x0 - x1;
  const double dy = y0 - y1;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

const char* ToString(LocalPlannerMode mode) {
  switch (mode) {
    case LocalPlannerMode::LOW_RISK_FOLLOW:
      return "LOW_RISK_FOLLOW";
    case LocalPlannerMode::HIGH_RISK_STRICT:
      return "HIGH_RISK_STRICT";
    case LocalPlannerMode::BLOCKED_RECOVERY:
      return "BLOCKED_RECOVERY";
  }
  return "UNKNOWN_MODE";
}

const char* ToString(LocalPlannerStatus status) {
  switch (status) {
    case LocalPlannerStatus::OK:
      return "OK";
    case LocalPlannerStatus::BLOCKED:
      return "BLOCKED";
    case LocalPlannerStatus::DIVERGED:
      return "DIVERGED";
    case LocalPlannerStatus::NEED_REPLAN:
      return "NEED_REPLAN";
    case LocalPlannerStatus::NO_REFERENCE:
      return "NO_REFERENCE";
  }
  return "UNKNOWN_STATUS";
}

RiskAwareLocalPlanner::RiskAwareLocalPlanner(const GridMap& map,
                                             const CollisionChecker& checker,
                                             const FootprintModel& footprint)
    : map_(&map), checker_(&checker), footprint_(&footprint) {}

void RiskAwareLocalPlanner::SetLocalObstacleMap(
    const GridMap* map, const CollisionChecker* checker) {
  local_obstacle_map_ = map;
  local_obstacle_checker_ = checker;
}

const GridMap& RiskAwareLocalPlanner::ActiveMap() const {
  if (use_dynamic_obstacles_for_cycle_ &&
      local_obstacle_map_ != nullptr && local_obstacle_map_->ready()) {
    return *local_obstacle_map_;
  }
  return *map_;
}

const CollisionChecker& RiskAwareLocalPlanner::ActiveChecker() const {
  if (use_dynamic_obstacles_for_cycle_ &&
      local_obstacle_map_ != nullptr && local_obstacle_map_->ready() &&
      local_obstacle_checker_ != nullptr) {
    return *local_obstacle_checker_;
  }
  return *checker_;
}

bool RiskAwareLocalPlanner::Initialize(ros::NodeHandle& pnh) {
  pnh.param("local_planner/window_length", window_length_, 3.0);
  pnh.param("local_planner/window_resample_spacing", window_resample_spacing_, 0.25);
  pnh.param("local_planner/high_risk_ratio_threshold", high_risk_ratio_threshold_, 0.35);
  pnh.param("local_planner/low_risk_smoothing_alpha", low_risk_smoothing_alpha_, 0.35);
  pnh.param("local_planner/high_risk_lateral_replan_threshold",
            high_risk_lateral_replan_threshold_, 0.35);
  pnh.param("local_planner/high_risk_yaw_replan_threshold",
            high_risk_yaw_replan_threshold_, 0.45);
  pnh.param("local_planner/strict_reference_deviation_threshold",
            strict_reference_deviation_threshold_, 0.40);
  pnh.param("local_planner/low_risk_clearance_margin", low_risk_clearance_margin_, 0.10);
  pnh.param("local_planner/high_risk_clearance_margin", high_risk_clearance_margin_, 0.05);
  pnh.param("local_planner/clearance_target_cap", clearance_target_cap_, 0.35);
  pnh.param("local_planner/push_step_size", push_step_size_, 0.05);
  pnh.param("local_planner/max_linear_speed", max_linear_speed_, 0.35);
  pnh.param("local_planner/max_angular_speed", max_angular_speed_, 0.8);
  pnh.param("local_planner/max_linear_accel", max_linear_accel_, 0.8);
  pnh.param("local_planner/max_angular_accel", max_angular_accel_, 0.8);
  pnh.param("local_planner/min_turn_linear_speed", min_turn_linear_speed_, 0.08);
  pnh.param("local_planner/min_turning_radius", min_turning_radius_, 0.75);
  pnh.param("local_planner/angular_gain", angular_gain_, 1.2);
  pnh.param("local_planner/sharp_turn_yaw_error", sharp_turn_yaw_error_, 0.65);
  pnh.param("local_planner/full_speed_yaw_error", full_speed_yaw_error_, 0.20);
  pnh.param("local_planner/pure_pursuit_lookahead", pure_pursuit_lookahead_, 0.80);
  pnh.param("local_planner/pure_pursuit_lateral_deadband",
            pure_pursuit_lateral_deadband_, 0.03);
  pnh.param("local_planner/angular_deadband", angular_deadband_, 0.03);
  pnh.param("local_planner/command_smoothing_alpha", command_smoothing_alpha_, 0.35);
  pnh.param("local_planner/dynamic_obstacle_relevance_radius",
            dynamic_obstacle_relevance_radius_, 0.65);
  pnh.param("local_planner/push_max_iters", push_max_iters_, 8);
  pnh.param("local_planner/consecutive_failure_limit", consecutive_failure_limit_, 3);
  pnh.param("local_planner/blocked_window_limit", blocked_window_limit_, 2);
  pnh.param("local_planner/horizon_time", horizon_time_, 2.5);
  pnh.param("local_planner/simulation_dt", simulation_dt_, 0.10);
  pnh.param("local_planner/control_period", control_period_, 0.10);
  pnh.param("local_planner/velocity_samples", velocity_samples_, 5);
  pnh.param("local_planner/curvature_samples", curvature_samples_, 9);
  pnh.param("local_planner/target_curvature_samples", target_curvature_samples_, 6);
  pnh.param("local_planner/allow_reverse", allow_reverse_, false);
  pnh.param("local_planner/goal_reached_xy_tolerance", goal_reached_xy_tolerance_, 0.25);
  pnh.param("local_planner/goal_reached_yaw_tolerance", goal_reached_yaw_tolerance_, 0.35);
  pnh.param("local_planner/low_risk_reference_weight", low_risk_reference_weight_, 2.0);
  pnh.param("local_planner/high_risk_reference_weight", high_risk_reference_weight_, 8.0);
  pnh.param("local_planner/yaw_weight", yaw_weight_, 0.4);
  pnh.param("local_planner/obstacle_weight", obstacle_weight_, 3.0);
  pnh.param("local_planner/progress_weight", progress_weight_, 2.5);
  pnh.param("local_planner/speed_weight", speed_weight_, 0.8);
  pnh.param("local_planner/curvature_weight", curvature_weight_, 0.15);
  pnh.param("local_planner/acceleration_weight", acceleration_weight_, 0.8);
  pnh.param("local_planner/angular_acceleration_weight",
            angular_acceleration_weight_, 1.0);
  horizon_time_ = std::max(0.5, horizon_time_);
  simulation_dt_ = std::max(0.02, simulation_dt_);
  control_period_ = std::max(0.02, control_period_);
  velocity_samples_ = std::max(2, velocity_samples_);
  curvature_samples_ = std::max(3, curvature_samples_);
  target_curvature_samples_ = std::max(0, target_curvature_samples_);
  min_turning_radius_ = std::max(1e-3, min_turning_radius_);
  max_angular_accel_ = std::max(0.05, max_angular_accel_);
  pure_pursuit_lookahead_ = std::max(0.10, pure_pursuit_lookahead_);
  pure_pursuit_lateral_deadband_ =
      std::max(0.0, pure_pursuit_lateral_deadband_);
  angular_deadband_ = std::max(0.0, angular_deadband_);
  command_smoothing_alpha_ =
      std::max(0.0, std::min(1.0, command_smoothing_alpha_));
  dynamic_obstacle_relevance_radius_ =
      std::max(0.05, dynamic_obstacle_relevance_radius_);
  clearance_target_cap_ = std::max(clearance_target_cap_,
                                   std::max(footprint_->inscribedRadius(),
                                            ActiveMap().resolution()));
  return map_ != nullptr && checker_ != nullptr && footprint_ != nullptr;
}

LocalPlanningResult RiskAwareLocalPlanner::Plan(
    const LocalPlannerState& current_state,
    const RiskAwareGlobalReferenceData& global_ref) {
  LocalPlanningResult result;
  if (map_ == nullptr || checker_ == nullptr || footprint_ == nullptr ||
      global_ref.points.empty()) {
    result.status = LocalPlannerStatus::NO_REFERENCE;
    result.debug_reason = "missing map/checker/reference";
    return result;
  }

  const int nearest_index = FindNearestReferenceIndex(current_state, global_ref);
  if (nearest_index < 0) {
    result.status = LocalPlannerStatus::NO_REFERENCE;
    result.debug_reason = "failed to match current pose to global reference";
    return result;
  }

  if (IsGoalReached(current_state, global_ref)) {
    result.success = true;
    result.has_command = true;
    result.status = LocalPlannerStatus::OK;
    result.mode = LocalPlannerMode::LOW_RISK_FOLLOW;
    result.matched_index = nearest_index;
    result.local_path.push_back(SE2State(current_state.x, current_state.y,
                                         current_state.yaw));
    result.local_cmd.linear_velocity = 0.0;
    result.local_cmd.angular_velocity = 0.0;
    result.debug_reason = "goal reached";
    last_command_ = result.local_cmd;
    has_last_command_ = true;
    consecutive_failures_ = 0;
    consecutive_blocked_cycles_ = 0;
    return result;
  }

  double high_risk_ratio = 0.0;
  result.local_reference =
      ExtractLocalReferenceWindow(global_ref, nearest_index, &high_risk_ratio);
  result.matched_index = nearest_index;
  if (result.local_reference.size() < 2) {
    result.status = LocalPlannerStatus::NO_REFERENCE;
    result.debug_reason = "local reference window too short";
    return result;
  }

  const GlobalReferencePointData& matched = global_ref.points[nearest_index];
  result.tracking_error = std::abs(SignedLateralError(current_state, matched));
  result.yaw_error = std::abs(WrappedYawError(current_state.yaw, matched.yaw));
  use_dynamic_obstacles_for_cycle_ =
      HasRelevantDynamicObstacle(result.local_reference);
  result.mode = DetermineLocalPlannerMode(result.local_reference, high_risk_ratio,
                                          use_dynamic_obstacles_for_cycle_);

  if (use_dynamic_obstacles_for_cycle_ && matched.risk_level == 1 &&
      (result.tracking_error > high_risk_lateral_replan_threshold_ ||
       result.yaw_error > high_risk_yaw_replan_threshold_)) {
    ++consecutive_failures_;
    result.need_replan = true;
    result.status = LocalPlannerStatus::NEED_REPLAN;
    result.consecutive_failures = consecutive_failures_;
    result.debug_reason = "high-risk divergence from global reference";
    return result;
  }

  if (!BuildLocalTrajectory(current_state, result.local_reference, result.mode, &result)) {
    ++consecutive_failures_;
    ++consecutive_blocked_cycles_;
    result.consecutive_failures = consecutive_failures_;
    result.blocked = true;
    if (consecutive_failures_ >= consecutive_failure_limit_ ||
        consecutive_blocked_cycles_ >= blocked_window_limit_) {
      result.need_replan = true;
      result.status = LocalPlannerStatus::NEED_REPLAN;
      if (result.debug_reason.empty()) {
        result.debug_reason = "local planning failed repeatedly";
      }
    } else {
      result.status = LocalPlannerStatus::BLOCKED;
      if (result.debug_reason.empty()) {
        result.debug_reason = "local window blocked";
      }
    }
    last_command_ = LocalPlannerCommand();
    has_last_command_ = true;
    return result;
  }

  if (!EvaluateLocalTrajectory(result.local_path, result.local_reference, &result)) {
    ++consecutive_failures_;
    ++consecutive_blocked_cycles_;
    result.consecutive_failures = consecutive_failures_;
    result.blocked = true;
    result.need_replan =
        consecutive_failures_ >= consecutive_failure_limit_ ||
        consecutive_blocked_cycles_ >= blocked_window_limit_;
    result.status = result.need_replan ? LocalPlannerStatus::NEED_REPLAN
                                       : LocalPlannerStatus::BLOCKED;
    if (result.debug_reason.empty()) {
      result.debug_reason = result.need_replan ? "trajectory validation failed repeatedly"
                                               : "trajectory validation failed";
    }
    last_command_ = LocalPlannerCommand();
    has_last_command_ = true;
    return result;
  }

  result.need_replan = CheckNeedReplan(current_state, result.local_reference, result);
  result.success = !result.local_path.empty() && !result.need_replan;
  result.status = result.need_replan ? LocalPlannerStatus::NEED_REPLAN
                                     : LocalPlannerStatus::OK;
  result.consecutive_failures = result.need_replan ? consecutive_failures_ + 1 : 0;
  if (result.need_replan) {
    ++consecutive_failures_;
    if (result.debug_reason.empty()) {
      result.debug_reason = "strict local path deviates from high-risk reference";
    }
    last_command_ = LocalPlannerCommand();
    has_last_command_ = true;
    return result;
  }

  consecutive_failures_ = 0;
  consecutive_blocked_cycles_ = 0;
  result.consecutive_failures = 0;
  if (!result.has_command) {
    UpdateCommand(current_state, result.local_reference, &result);
  }
  last_command_ = result.local_cmd;
  has_last_command_ = true;
  return result;
}

std::vector<GlobalReferencePointData> RiskAwareLocalPlanner::ExtractLocalReferenceWindow(
    const RiskAwareGlobalReferenceData& global_ref, int nearest_index,
    double* high_risk_ratio) const {
  std::vector<GlobalReferencePointData> window;
  if (nearest_index < 0 || nearest_index >= static_cast<int>(global_ref.points.size())) {
    return window;
  }

  const double start_s = global_ref.points[nearest_index].s;
  double next_emit_s = start_s;
  int high_risk_count = 0;

  // Build a short forward-looking reference window in arc-length space. This is
  // the local planner's direct input and corresponds to Section 3.4.1 in the
  // paper.
  for (size_t i = static_cast<size_t>(nearest_index); i < global_ref.points.size(); ++i) {
    const GlobalReferencePointData& point = global_ref.points[i];
    if (point.s > start_s + window_length_ + 1e-6) {
      break;
    }
    if (!window.empty() && point.s + 1e-6 < next_emit_s &&
        i + 1 < global_ref.points.size()) {
      continue;
    }
    window.push_back(point);
    next_emit_s = point.s + window_resample_spacing_;
    if (point.risk_level == 1) {
      ++high_risk_count;
    }
  }

  if (window.size() == 1 && nearest_index + 1 < static_cast<int>(global_ref.points.size())) {
    window.push_back(global_ref.points[nearest_index + 1]);
    if (global_ref.points[nearest_index + 1].risk_level == 1) {
      ++high_risk_count;
    }
  }

  if (high_risk_ratio != nullptr) {
    *high_risk_ratio = window.empty()
                           ? 0.0
                           : static_cast<double>(high_risk_count) /
                                 static_cast<double>(window.size());
  }
  return window;
}

LocalPlannerMode RiskAwareLocalPlanner::DetermineLocalPlannerMode(
    const std::vector<GlobalReferencePointData>& window,
    double high_risk_ratio,
    bool dynamic_obstacle_relevant) const {
  if (window.empty()) {
    return LocalPlannerMode::BLOCKED_RECOVERY;
  }
  if (high_risk_ratio >= high_risk_ratio_threshold_ ||
      window.front().risk_level == 1) {
    return LocalPlannerMode::HIGH_RISK_STRICT;
  }
  if (consecutive_failures_ > 0 || consecutive_blocked_cycles_ > 0) {
    return LocalPlannerMode::BLOCKED_RECOVERY;
  }
  if (dynamic_obstacle_relevant) {
    return LocalPlannerMode::BLOCKED_RECOVERY;
  }
  return LocalPlannerMode::LOW_RISK_FOLLOW;
}

bool RiskAwareLocalPlanner::HasRelevantDynamicObstacle(
    const std::vector<GlobalReferencePointData>& window) const {
  if (window.empty() || local_obstacle_map_ == nullptr ||
      !local_obstacle_map_->ready() || map_ == nullptr || !map_->ready()) {
    return false;
  }

  for (const GlobalReferencePointData& point : window) {
    if (HasDynamicObstacleNear(point.x, point.y,
                               dynamic_obstacle_relevance_radius_)) {
      return true;
    }
  }
  return false;
}

bool RiskAwareLocalPlanner::HasDynamicObstacleNear(
    double wx, double wy, double radius) const {
  if (local_obstacle_map_ == nullptr || !local_obstacle_map_->ready() ||
      map_ == nullptr || !map_->ready()) {
    return false;
  }

  const GridIndex center = local_obstacle_map_->worldToGrid(wx, wy);
  const int radius_cells =
      std::max(1, static_cast<int>(std::ceil(
                      radius / std::max(1e-3, local_obstacle_map_->resolution()))));
  for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      if (dx * dx + dy * dy > radius_cells * radius_cells) {
        continue;
      }
      if (IsNewDynamicObstacleCell(center.x + dx, center.y + dy)) {
        return true;
      }
    }
  }
  return false;
}

bool RiskAwareLocalPlanner::IsNewDynamicObstacleCell(int gx, int gy) const {
  if (local_obstacle_map_ == nullptr || !local_obstacle_map_->ready() ||
      map_ == nullptr || !map_->ready()) {
    return false;
  }
  if (!local_obstacle_map_->isInside(gx, gy) ||
      !local_obstacle_map_->isRawOccupied(gx, gy)) {
    return false;
  }

  const Eigen::Vector2d world = local_obstacle_map_->gridToWorld(gx, gy);
  const GridIndex static_index = map_->worldToGrid(world.x(), world.y());
  if (!map_->isInside(static_index.x, static_index.y)) {
    return false;
  }
  return !map_->isRawOccupied(static_index.x, static_index.y);
}

bool RiskAwareLocalPlanner::BuildLocalTrajectory(
    const LocalPlannerState& current_state,
    const std::vector<GlobalReferencePointData>& window,
    LocalPlannerMode mode, LocalPlanningResult* result) const {
  if (result == nullptr || window.size() < 2) {
    return false;
  }

  const GridMap& active_map = ActiveMap();
  const CollisionChecker& active_checker = ActiveChecker();
  std::vector<SE2State> path;
  path.reserve(window.size() + 1);
  path.push_back(SE2State(current_state.x, current_state.y, current_state.yaw));

  for (size_t i = 0; i < window.size(); ++i) {
    const GlobalReferencePointData& point = window[i];
    Eigen::Vector2d target(point.x, point.y);
    if (mode == LocalPlannerMode::LOW_RISK_FOLLOW && i > 0 &&
        i + 1 < window.size()) {
      const Eigen::Vector2d prev(window[i - 1].x, window[i - 1].y);
      const Eigen::Vector2d next(window[i + 1].x, window[i + 1].y);
      const Eigen::Vector2d smoothed = 0.25 * prev + 0.50 * target + 0.25 * next;
      target = (1.0 - low_risk_smoothing_alpha_) * target +
               low_risk_smoothing_alpha_ * smoothed;
    }

    const double target_clearance = TargetClearance(mode, point);
    const double reference_yaw = point.yaw;
    const SE2State initial_guess(target.x(), target.y(), reference_yaw);
    bool point_safe =
        active_map.getEsdf(target.x(), target.y()) > target_clearance &&
        active_checker.isFree(initial_guess);
    if (!point_safe) {
      point_safe = math::PushPointFromObstacle(active_map, target,
                                               target_clearance,
                                               push_step_size_,
                                               push_max_iters_);
      if (!point_safe ||
          !active_checker.isFree(SE2State(target.x(), target.y(),
                                          reference_yaw))) {
        result->debug_reason = "unable to push local waypoint out of collision";
        return false;
      }
    }

    if (mode == LocalPlannerMode::HIGH_RISK_STRICT &&
        Distance2d(target.x(), target.y(), point.x, point.y) >
            strict_reference_deviation_threshold_) {
      result->debug_reason =
          "strict mode exceeded reference deviation threshold";
      return false;
    }

    double yaw = reference_yaw;
    if (mode != LocalPlannerMode::HIGH_RISK_STRICT &&
        Distance2d(path.back().x, path.back().y, target.x(), target.y()) > 1e-4) {
      yaw = std::atan2(target.y() - path.back().y,
                      target.x() - path.back().x);
    }
    path.push_back(SE2State(target.x(), target.y(), yaw));
  }

  if (path.size() < 2) {
    result->debug_reason = "local path has insufficient support points";
    return false;
  }

  path.front().yaw = current_state.yaw;
  for (size_t i = 1; i + 1 < path.size(); ++i) {
    const Eigen::Vector2d delta = path[i + 1].position() - path[i].position();
    if (delta.norm() > 1e-5) {
      path[i].yaw = std::atan2(delta.y(), delta.x());
    }
  }
  path.back().yaw = window.back().yaw;

  result->local_path = path;
  result->debug_reason =
      use_dynamic_obstacles_for_cycle_
          ? "dynamic obstacle avoidance active; reference path selected"
          : "static reference tracking; reference path selected";
  return true;
}

bool RiskAwareLocalPlanner::EvaluateLocalTrajectory(
    const std::vector<SE2State>& path,
    const std::vector<GlobalReferencePointData>& window,
    LocalPlanningResult* result) const {
  if (result == nullptr || path.size() < 2) {
    return false;
  }

  const GridMap& active_map = ActiveMap();
  const CollisionChecker& active_checker = ActiveChecker();
  result->min_clearance = kInf;
  for (size_t i = 0; i < path.size(); ++i) {
    const double clearance = active_map.getEsdf(path[i].x, path[i].y);
    result->min_clearance = std::min(result->min_clearance, clearance);
    if (!std::isfinite(clearance) || !active_checker.isFree(path[i])) {
      result->debug_reason = "local path endpoint collides";
      return false;
    }
  }

  const double line_step = std::max(0.05, 0.5 * active_map.resolution());
  for (size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector2d delta = path[i].position() - path[i - 1].position();
    const double distance = delta.norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / line_step)));
    for (int s = 0; s <= samples; ++s) {
      const double alpha = static_cast<double>(s) / static_cast<double>(samples);
      const Eigen::Vector2d interp = path[i - 1].position() + alpha * delta;
      const double yaw =
          std::atan2(std::sin((1.0 - alpha) * path[i - 1].yaw + alpha * path[i].yaw),
                     std::cos((1.0 - alpha) * path[i - 1].yaw + alpha * path[i].yaw));
      const SE2State sample(interp.x(), interp.y(), yaw);
      const double clearance = active_map.getEsdf(interp.x(), interp.y());
      result->min_clearance = std::min(result->min_clearance, clearance);
      if (!std::isfinite(clearance) || !active_checker.isFree(sample)) {
        result->debug_reason = "local path segment collides";
        return false;
      }
    }
  }

  if (!window.empty()) {
    // The local path must preserve at least the clearance target implied by the
    // current risk-aware reference window.
    const double required_clearance =
        std::max(0.0, TargetClearance(result->mode, window.front()));
    if (result->min_clearance <= required_clearance - 1e-3) {
      result->debug_reason = "trajectory clearance below local target";
      return false;
    }
  }
  return true;
}

bool RiskAwareLocalPlanner::CheckNeedReplan(
    const LocalPlannerState& current_state,
    const std::vector<GlobalReferencePointData>& window,
    const LocalPlanningResult& partial) const {
  if (window.empty()) {
    return true;
  }

  const GlobalReferencePointData& head = window.front();
  if (head.risk_level == 1) {
    // In a high-risk prefix, lateral and yaw deviation are treated as direct
    // replan triggers rather than waiting for full local failure.
    const double lateral = std::abs(SignedLateralError(current_state, head));
    const double yaw_error = std::abs(WrappedYawError(current_state.yaw, head.yaw));
    if (lateral > high_risk_lateral_replan_threshold_ ||
        yaw_error > high_risk_yaw_replan_threshold_) {
      return true;
    }
  }

  if (partial.mode == LocalPlannerMode::HIGH_RISK_STRICT &&
      partial.local_path.size() > 1) {
    // Strict mode also checks whether the constructed local path drifts too far
    // away from the risk-aware reference itself.
    for (size_t i = 1; i < partial.local_path.size(); ++i) {
      double distance = kInf;
      const int ref_index =
          FindNearestReferenceIndexInWindow(partial.local_path[i], window, &distance);
      if (ref_index < 0 || distance > strict_reference_deviation_threshold_) {
        return true;
      }
    }
  }

  return false;
}

bool RiskAwareLocalPlanner::IsGoalReached(
    const LocalPlannerState& current_state,
    const RiskAwareGlobalReferenceData& global_ref) const {
  if (global_ref.points.empty()) {
    return false;
  }
  const GlobalReferencePointData& goal = global_ref.points.back();
  const double xy_error =
      Distance2d(current_state.x, current_state.y, goal.x, goal.y);
  const double yaw_error = std::abs(WrappedYawError(current_state.yaw, goal.yaw));
  return xy_error <= goal_reached_xy_tolerance_ &&
         yaw_error <= goal_reached_yaw_tolerance_;
}

int RiskAwareLocalPlanner::FindNearestReferenceIndex(
    const LocalPlannerState& current_state,
    const RiskAwareGlobalReferenceData& global_ref) const {
  int best_index = -1;
  double best_distance = kInf;
  for (size_t i = 0; i < global_ref.points.size(); ++i) {
    const double distance = Distance2d(current_state.x, current_state.y,
                                       global_ref.points[i].x,
                                       global_ref.points[i].y);
    if (distance < best_distance) {
      best_distance = distance;
      best_index = static_cast<int>(i);
    }
  }
  return best_index;
}

int RiskAwareLocalPlanner::FindNearestReferenceIndexInWindow(
    const SE2State& state,
    const std::vector<GlobalReferencePointData>& window,
    double* distance) const {
  int best_index = -1;
  double best_distance = kInf;
  for (size_t i = 0; i < window.size(); ++i) {
    const double d = Distance2d(state.x, state.y, window[i].x, window[i].y);
    if (d < best_distance) {
      best_distance = d;
      best_index = static_cast<int>(i);
    }
  }
  if (distance != nullptr) {
    *distance = best_distance;
  }
  return best_index;
}

double RiskAwareLocalPlanner::PreferredSpeedForWindow(
    const std::vector<GlobalReferencePointData>& window,
    LocalPlannerMode mode) const {
  if (window.empty()) {
    return 0.0;
  }
  double preferred_speed = 0.0;
  for (const GlobalReferencePointData& point : window) {
    preferred_speed += point.preferred_speed;
  }
  preferred_speed /= static_cast<double>(window.size());
  if (mode == LocalPlannerMode::HIGH_RISK_STRICT) {
    preferred_speed *= 0.8;
  } else if (mode == LocalPlannerMode::BLOCKED_RECOVERY) {
    preferred_speed *= 0.5;
  }
  return std::max(0.0, std::min(max_linear_speed_, preferred_speed));
}

std::vector<double> RiskAwareLocalPlanner::BuildSpeedSamples(
    const LocalPlannerState& current_state,
    const std::vector<GlobalReferencePointData>& window,
    LocalPlannerMode mode) const {
  const double current_v =
      current_state.has_velocity
          ? current_state.v
          : (has_last_command_ ? last_command_.linear_velocity : 0.0);
  const double preferred_speed = PreferredSpeedForWindow(window, mode);
  const double accel_span = std::max(0.02, max_linear_accel_ * control_period_);
  const double physical_min_v = allow_reverse_ ? -max_linear_speed_ : 0.0;
  const double low = std::max(physical_min_v, current_v - accel_span);
  const double high = std::min(max_linear_speed_, current_v + accel_span);

  std::vector<double> samples;
  samples.reserve(static_cast<size_t>(velocity_samples_) + 4);
  if (velocity_samples_ == 1 || std::abs(high - low) < 1e-6) {
    samples.push_back(std::max(low, std::min(high, preferred_speed)));
  } else {
    for (int i = 0; i < velocity_samples_; ++i) {
      const double alpha =
          static_cast<double>(i) / static_cast<double>(velocity_samples_ - 1);
      samples.push_back(low + alpha * (high - low));
    }
  }
  samples.push_back(std::max(low, std::min(high, preferred_speed)));
  samples.push_back(std::max(low, std::min(high, current_v)));
  samples.push_back(std::max(low, std::min(high, min_turn_linear_speed_)));
  samples.push_back(std::max(low, std::min(high, 0.0)));

  std::sort(samples.begin(), samples.end());
  samples.erase(std::unique(samples.begin(), samples.end(),
                            [](double a, double b) {
                              return std::abs(a - b) < 1e-4;
                            }),
                samples.end());
  return samples;
}

std::vector<double> RiskAwareLocalPlanner::BuildCurvatureSamples(
    const LocalPlannerState& current_state,
    const std::vector<GlobalReferencePointData>& window) const {
  const double max_curvature = 1.0 / std::max(1e-3, min_turning_radius_);
  std::vector<double> samples;
  samples.reserve(static_cast<size_t>(curvature_samples_ +
                                      target_curvature_samples_ + 1));
  for (int i = 0; i < curvature_samples_; ++i) {
    const double alpha =
        curvature_samples_ == 1
            ? 0.5
            : static_cast<double>(i) / static_cast<double>(curvature_samples_ - 1);
    samples.push_back(-max_curvature + 2.0 * max_curvature * alpha);
  }
  samples.push_back(0.0);

  const SE2State current(current_state.x, current_state.y, current_state.yaw);
  const int target_count =
      std::min(target_curvature_samples_, static_cast<int>(window.size()));
  for (int i = 0; i < target_count; ++i) {
    const size_t index = static_cast<size_t>(i);
    const double dx = window[index].x - current.x;
    const double dy = window[index].y - current.y;
    const double lookahead = std::hypot(dx, dy);
    if (lookahead < 1e-3) {
      continue;
    }
    const double heading = std::atan2(dy, dx);
    const double alpha = normalizeAngle(heading - current.yaw);
    const double curvature = 2.0 * std::sin(alpha) / lookahead;
    samples.push_back(std::max(-max_curvature,
                               std::min(max_curvature, curvature)));
  }

  std::sort(samples.begin(), samples.end());
  samples.erase(std::unique(samples.begin(), samples.end(),
                            [](double a, double b) {
                              return std::abs(a - b) < 1e-4;
                            }),
                samples.end());
  return samples;
}

bool RiskAwareLocalPlanner::RolloutAckermannCandidate(
    const LocalPlannerState& current_state, double linear_velocity,
    double curvature, LocalPlannerMode mode,
    const std::vector<GlobalReferencePointData>& window,
    std::vector<SE2State>* path, double* cost,
    std::string* reject_reason) const {
  if (path == nullptr || cost == nullptr || window.empty()) {
    return false;
  }

  const double current_v =
      current_state.has_velocity
          ? current_state.v
          : (has_last_command_ ? last_command_.linear_velocity : 0.0);
  const double preferred_speed = PreferredSpeedForWindow(window, mode);
  const double reference_weight =
      mode == LocalPlannerMode::HIGH_RISK_STRICT
          ? high_risk_reference_weight_
          : low_risk_reference_weight_;
  const double target_clearance_margin =
      mode == LocalPlannerMode::HIGH_RISK_STRICT
          ? high_risk_clearance_margin_
          : low_risk_clearance_margin_;
  const GridMap& active_map = ActiveMap();
  const CollisionChecker& active_checker = ActiveChecker();

  SE2State state(current_state.x, current_state.y, current_state.yaw);
  path->clear();
  path->reserve(static_cast<size_t>(std::ceil(horizon_time_ / simulation_dt_)) + 1);
  path->push_back(state);

  double total_cost = 0.0;
  double max_progress = 0.0;
  double min_clearance = kInf;
  const double start_s = window.front().s;
  const int steps = std::max(1, static_cast<int>(std::ceil(horizon_time_ / simulation_dt_)));
  const double omega =
      std::max(-max_angular_speed_,
               std::min(max_angular_speed_, linear_velocity * curvature));

  if (std::abs(linear_velocity) < 1e-3) {
    if (reject_reason != nullptr) {
      *reject_reason = "ackermann rollout does not advance";
    }
    return false;
  }

  for (int step = 0; step < steps; ++step) {
    state.x += linear_velocity * std::cos(state.yaw) * simulation_dt_;
    state.y += linear_velocity * std::sin(state.yaw) * simulation_dt_;
    state.yaw = normalizeAngle(state.yaw + omega * simulation_dt_);

    double ref_distance = kInf;
    const int ref_index = FindNearestReferenceIndexInWindow(state, window, &ref_distance);
    if (ref_index < 0) {
      if (reject_reason != nullptr) {
        *reject_reason = "rollout lost risk-aware reference";
      }
      return false;
    }
    const GlobalReferencePointData& ref = window[static_cast<size_t>(ref_index)];
    const double required_clearance = TargetClearance(mode, ref);
    const double clearance = active_map.getEsdf(state.x, state.y);
    min_clearance = std::min(min_clearance, clearance);

    if (!std::isfinite(clearance) || !active_checker.isFree(state)) {
      if (reject_reason != nullptr) {
        *reject_reason = "ackermann rollout collides";
      }
      return false;
    }
    if (clearance < required_clearance) {
      if (reject_reason != nullptr) {
        *reject_reason = "ackermann rollout violates required clearance";
      }
      return false;
    }
    if (mode == LocalPlannerMode::HIGH_RISK_STRICT &&
        ref_distance > strict_reference_deviation_threshold_) {
      if (reject_reason != nullptr) {
        *reject_reason = "strict rollout deviates from risk-aware reference";
      }
      return false;
    }

    const double yaw_error = WrappedYawError(state.yaw, ref.yaw);
    const double clearance_slack =
        std::max(1e-3, clearance - required_clearance);
    const double near_obstacle_cost =
        1.0 / (clearance_slack + target_clearance_margin + 1e-3);
    total_cost += reference_weight * ref_distance * ref_distance;
    total_cost += yaw_weight_ * yaw_error * yaw_error;
    total_cost += obstacle_weight_ * near_obstacle_cost;
    max_progress = std::max(max_progress, ref.s - start_s);
    path->push_back(state);
  }

  const SE2State& terminal = path->back();
  double terminal_distance = kInf;
  FindNearestReferenceIndexInWindow(terminal, window, &terminal_distance);
  total_cost += reference_weight * terminal_distance * terminal_distance;
  total_cost += speed_weight_ *
                (linear_velocity - preferred_speed) *
                (linear_velocity - preferred_speed);
  total_cost += curvature_weight_ * curvature * curvature;
  total_cost += acceleration_weight_ *
                (linear_velocity - current_v) * (linear_velocity - current_v);
  const double previous_omega =
      has_last_command_ ? last_command_.angular_velocity : 0.0;
  total_cost += angular_acceleration_weight_ *
                (omega - previous_omega) * (omega - previous_omega);
  total_cost -= progress_weight_ * max_progress;

  if (!std::isfinite(total_cost) || !std::isfinite(min_clearance)) {
    if (reject_reason != nullptr) {
      *reject_reason = "ackermann rollout produced invalid score";
    }
    return false;
  }

  *cost = total_cost;
  return true;
}

double RiskAwareLocalPlanner::SignedLateralError(
    const LocalPlannerState& current_state,
    const GlobalReferencePointData& ref_point) const {
  const double dx = current_state.x - ref_point.x;
  const double dy = current_state.y - ref_point.y;
  return -std::sin(ref_point.yaw) * dx + std::cos(ref_point.yaw) * dy;
}

double RiskAwareLocalPlanner::WrappedYawError(double yaw,
                                              double reference_yaw) const {
  return normalizeAngle(yaw - reference_yaw);
}

double RiskAwareLocalPlanner::TargetClearance(
    LocalPlannerMode mode, const GlobalReferencePointData& ref_point) const {
  const double base =
      std::max(footprint_->inscribedRadius(), ActiveMap().resolution());
  const double reference_clearance =
      std::isfinite(ref_point.clearance) ? ref_point.clearance : base;
  const double margin = mode == LocalPlannerMode::HIGH_RISK_STRICT
                            ? high_risk_clearance_margin_
                            : low_risk_clearance_margin_;
  const double target =
      std::min(std::max(base, reference_clearance - margin),
               std::max(reference_clearance, base));
  const double cap = std::max(base, clearance_target_cap_);
  if (!std::isfinite(target)) {
    return cap;
  }
  return std::min(target, cap);
}

void RiskAwareLocalPlanner::UpdateCommand(
    const LocalPlannerState& current_state,
    const std::vector<GlobalReferencePointData>& window,
    LocalPlanningResult* result) const {
  if (result == nullptr || result->local_path.size() < 2 || window.empty()) {
    return;
  }

  const SE2State current(current_state.x, current_state.y, current_state.yaw);
  SE2State lookahead = result->local_path.back();
  double accumulated = 0.0;
  for (size_t i = 1; i < result->local_path.size(); ++i) {
    accumulated += Distance2d(result->local_path[i - 1].x,
                              result->local_path[i - 1].y,
                              result->local_path[i].x,
                              result->local_path[i].y);
    if (accumulated >= pure_pursuit_lookahead_) {
      lookahead = result->local_path[i];
      break;
    }
  }

  const double dx = lookahead.x - current.x;
  const double dy = lookahead.y - current.y;
  const double cos_yaw = std::cos(current.yaw);
  const double sin_yaw = std::sin(current.yaw);
  const double x_local = cos_yaw * dx + sin_yaw * dy;
  const double y_local = -sin_yaw * dx + cos_yaw * dy;
  const double lookahead_sq = std::max(1e-4, dx * dx + dy * dy);
  const double lookahead_dist = std::sqrt(lookahead_sq);
  const double heading = std::atan2(dy, dx);
  const double yaw_error = WrappedYawError(heading, current_state.yaw);
  const double y_control =
      std::abs(y_local) < pure_pursuit_lateral_deadband_ ? 0.0 : y_local;
  const double curvature = 2.0 * y_control / lookahead_sq;

  double preferred_speed = PreferredSpeedForWindow(window, result->mode);

  const double abs_yaw_error = std::abs(yaw_error);
  double yaw_speed_scale = 1.0;
  if (abs_yaw_error >= sharp_turn_yaw_error_) {
    yaw_speed_scale = 0.0;
  } else if (abs_yaw_error > full_speed_yaw_error_) {
    const double denom =
        std::max(1e-3, sharp_turn_yaw_error_ - full_speed_yaw_error_);
    yaw_speed_scale = (sharp_turn_yaw_error_ - abs_yaw_error) / denom;
  }

  double linear_velocity = preferred_speed * yaw_speed_scale;
  if (abs_yaw_error > full_speed_yaw_error_ && preferred_speed > 1e-3) {
    linear_velocity = std::max(min_turn_linear_speed_, linear_velocity);
  }
  const bool target_behind = x_local < 0.0;
  const bool near_lookahead = lookahead_dist < 0.05;
  if (near_lookahead) {
    linear_velocity = 0.0;
  } else if (target_behind && preferred_speed > 1e-3) {
    linear_velocity = min_turn_linear_speed_;
  }

  const double curvature_speed_scale =
      1.0 / (1.0 + std::abs(curvature) * std::max(0.1, min_turning_radius_));
  if (!target_behind) {
    linear_velocity = linear_velocity * curvature_speed_scale;
  }
  linear_velocity = std::min(max_linear_speed_, linear_velocity);

  double angular_velocity = linear_velocity * curvature;
  if (target_behind || std::abs(linear_velocity) < 1e-3) {
    angular_velocity = angular_gain_ * yaw_error;
  } else {
    const double ackermann_angular_limit =
        std::abs(linear_velocity) / std::max(1e-3, min_turning_radius_);
    angular_velocity =
        std::max(-ackermann_angular_limit,
                 std::min(ackermann_angular_limit, angular_velocity));
  }
  angular_velocity =
      std::max(-max_angular_speed_, std::min(max_angular_speed_, angular_velocity));
  if (std::abs(angular_velocity) < angular_deadband_) {
    angular_velocity = 0.0;
  }

  result->local_cmd.linear_velocity = linear_velocity;
  result->local_cmd.angular_velocity = angular_velocity;
  result->has_command = true;
  ApplyCommandSmoothing(result);
}

void RiskAwareLocalPlanner::ApplyCommandSmoothing(LocalPlanningResult* result) const {
  if (result == nullptr || !result->has_command || !has_last_command_) {
    return;
  }

  const double alpha = command_smoothing_alpha_;
  result->local_cmd.linear_velocity =
      (1.0 - alpha) * last_command_.linear_velocity +
      alpha * result->local_cmd.linear_velocity;
  result->local_cmd.angular_velocity =
      (1.0 - alpha) * last_command_.angular_velocity +
      alpha * result->local_cmd.angular_velocity;
  const double max_delta = max_angular_accel_ * control_period_;
  const double angular_delta =
      result->local_cmd.angular_velocity - last_command_.angular_velocity;
  if (std::abs(angular_delta) > max_delta) {
    result->local_cmd.angular_velocity =
        last_command_.angular_velocity +
        std::copysign(max_delta, angular_delta);
  }
  if (std::abs(result->local_cmd.angular_velocity) < angular_deadband_) {
    result->local_cmd.angular_velocity = 0.0;
  }
}

}  // namespace isweep_planner
