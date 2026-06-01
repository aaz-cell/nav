#include "isweep_planner/local_planner/risk_aware_local_planner.h"

#include <algorithm>
#include <cmath>

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
  pnh.param("local_planner/push_step_size", push_step_size_, 0.05);
  pnh.param("local_planner/max_linear_speed", max_linear_speed_, 0.35);
  pnh.param("local_planner/max_angular_speed", max_angular_speed_, 0.8);
  pnh.param("local_planner/min_turn_linear_speed", min_turn_linear_speed_, 0.08);
  pnh.param("local_planner/min_turning_radius", min_turning_radius_, 0.75);
  pnh.param("local_planner/angular_gain", angular_gain_, 1.2);
  pnh.param("local_planner/sharp_turn_yaw_error", sharp_turn_yaw_error_, 0.65);
  pnh.param("local_planner/full_speed_yaw_error", full_speed_yaw_error_, 0.20);
  pnh.param("local_planner/push_max_iters", push_max_iters_, 8);
  pnh.param("local_planner/consecutive_failure_limit", consecutive_failure_limit_, 3);
  pnh.param("local_planner/blocked_window_limit", blocked_window_limit_, 2);
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
  result.mode = DetermineLocalPlannerMode(result.local_reference, high_risk_ratio);

  if (matched.risk_level == 1 &&
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
    return result;
  }

  consecutive_failures_ = 0;
  consecutive_blocked_cycles_ = 0;
  result.consecutive_failures = 0;
  UpdateCommand(current_state, result.local_reference, &result);
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
    double high_risk_ratio) const {
  if (window.empty()) {
    return LocalPlannerMode::BLOCKED_RECOVERY;
  }
  // High-risk mode is activated either by the local risk composition or when
  // the first reference point already lies in a bottleneck region.
  if (high_risk_ratio >= high_risk_ratio_threshold_ || window.front().risk_level == 1) {
    return LocalPlannerMode::HIGH_RISK_STRICT;
  }
  if (consecutive_failures_ > 0 || consecutive_blocked_cycles_ > 0) {
    return LocalPlannerMode::BLOCKED_RECOVERY;
  }
  return LocalPlannerMode::LOW_RISK_FOLLOW;
}

bool RiskAwareLocalPlanner::BuildLocalTrajectory(
    const LocalPlannerState& current_state,
    const std::vector<GlobalReferencePointData>& window,
    LocalPlannerMode mode, LocalPlanningResult* result) const {
  if (result == nullptr || window.size() < 2) {
    return false;
  }

  std::vector<SE2State> path;
  path.reserve(window.size() + 1);
  path.push_back(SE2State(current_state.x, current_state.y, current_state.yaw));

  for (size_t i = 0; i < window.size(); ++i) {
    const GlobalReferencePointData& point = window[i];
    Eigen::Vector2d target(point.x, point.y);
    // Low-risk mode is allowed to smooth the reference slightly so local motion
    // is less nervous in open space. High-risk mode keeps the reference strict.
    if (mode == LocalPlannerMode::LOW_RISK_FOLLOW && i > 0 && i + 1 < window.size()) {
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
        map_->getEsdf(target.x(), target.y()) > target_clearance &&
        checker_->isFree(initial_guess);
    if (!point_safe) {
      // If the sampled reference is too close to obstacles, try a short-range
      // repair before declaring the local window blocked.
      point_safe = math::PushPointFromObstacle(*map_, target, target_clearance,
                                              push_step_size_, push_max_iters_);
      if (!point_safe ||
          !checker_->isFree(SE2State(target.x(), target.y(), reference_yaw))) {
        result->debug_reason = "unable to push local waypoint out of collision";
        return false;
      }
    }

    if (mode == LocalPlannerMode::HIGH_RISK_STRICT &&
        Distance2d(target.x(), target.y(), point.x, point.y) >
            strict_reference_deviation_threshold_) {
      result->debug_reason = "strict mode exceeded risk-aware reference deviation threshold";
      return false;
    }

    double yaw = reference_yaw;
    // Outside strict mode the local path may align yaw with its immediate
    // geometric direction for smoother execution.
    if (mode != LocalPlannerMode::HIGH_RISK_STRICT &&
        Distance2d(path.back().x, path.back().y, target.x(), target.y()) > 1e-4) {
      yaw = std::atan2(target.y() - path.back().y, target.x() - path.back().x);
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
  return true;
}

bool RiskAwareLocalPlanner::EvaluateLocalTrajectory(
    const std::vector<SE2State>& path,
    const std::vector<GlobalReferencePointData>& window,
    LocalPlanningResult* result) const {
  if (result == nullptr || path.size() < 2) {
    return false;
  }

  result->min_clearance = kInf;
  for (size_t i = 0; i < path.size(); ++i) {
    const double clearance = map_->getEsdf(path[i].x, path[i].y);
    result->min_clearance = std::min(result->min_clearance, clearance);
    if (!std::isfinite(clearance) || !checker_->isFree(path[i])) {
      result->debug_reason = "local path endpoint collides";
      return false;
    }
  }

  const double line_step = std::max(0.05, 0.5 * map_->resolution());
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
      const double clearance = map_->getEsdf(interp.x(), interp.y());
      result->min_clearance = std::min(result->min_clearance, clearance);
      if (!std::isfinite(clearance) || !checker_->isFree(sample)) {
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
    size_t compare_count = std::min(partial.local_path.size() - 1, window.size());
    for (size_t i = 0; i < compare_count; ++i) {
      const SE2State& local = partial.local_path[i + 1];
      const GlobalReferencePointData& ref = window[i];
      if (Distance2d(local.x, local.y, ref.x, ref.y) >
          strict_reference_deviation_threshold_) {
        return true;
      }
    }
  }

  return false;
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
  const double base = std::max(footprint_->inscribedRadius(), map_->resolution());
  const double reference_clearance =
      std::isfinite(ref_point.clearance) ? ref_point.clearance : base;
  // High-risk mode keeps the allowed clearance band tighter to avoid drifting
  // into unsafe free space estimates near bottlenecks.
  if (mode == LocalPlannerMode::HIGH_RISK_STRICT) {
    return std::min(std::max(base, reference_clearance - high_risk_clearance_margin_),
                    reference_clearance);
  }
  return std::min(std::max(base, reference_clearance - low_risk_clearance_margin_),
                  std::max(reference_clearance, base));
}

void RiskAwareLocalPlanner::UpdateCommand(
    const LocalPlannerState& current_state,
    const std::vector<GlobalReferencePointData>& window,
    LocalPlanningResult* result) const {
  if (result == nullptr || result->local_path.size() < 2 || window.empty()) {
    return;
  }

  const SE2State& lookahead =
      result->local_path[std::min<size_t>(2, result->local_path.size() - 1)];
  const double heading =
      std::atan2(lookahead.y - current_state.y, lookahead.x - current_state.x);
  const double yaw_error = WrappedYawError(heading, current_state.yaw);

  double preferred_speed = 0.0;
  for (size_t i = 0; i < window.size(); ++i) {
    preferred_speed += window[i].preferred_speed;
  }
  preferred_speed /= static_cast<double>(window.size());
  // Risk semantics are finally converted into a simple executable command:
  // strict windows slow down, blocked-recovery windows slow down further.
  if (result->mode == LocalPlannerMode::HIGH_RISK_STRICT) {
    preferred_speed *= 0.8;
  } else if (result->mode == LocalPlannerMode::BLOCKED_RECOVERY) {
    preferred_speed *= 0.5;
  }

  preferred_speed = std::max(0.0, std::min(max_linear_speed_, preferred_speed));

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
  linear_velocity = std::min(max_linear_speed_, linear_velocity);
  const double ackermann_angular_limit =
      std::abs(linear_velocity) / std::max(1e-3, min_turning_radius_);
  const double angular_limit = std::min(max_angular_speed_, ackermann_angular_limit);

  result->local_cmd.linear_velocity = linear_velocity;
  result->local_cmd.angular_velocity =
      std::max(-angular_limit, std::min(angular_limit, angular_gain_ * yaw_error));
}

}  // namespace isweep_planner
