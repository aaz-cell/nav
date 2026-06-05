#pragma once

#include "isweep_planner/core/math_utils.h"
#include "isweep_planner/env/collision_checker.h"
#include "isweep_planner/env/footprint_model.h"
#include "isweep_planner/env/grid_map.h"
#include "isweep_planner/local_planner/local_planner_interface.h"

namespace isweep_planner {

// Minimal local planner that explicitly tracks the risk-aware global reference.
// It is intentionally lightweight: window extraction + risk-adaptive tracking +
// simple ESDF push-out. Future New-PAD integration can replace BuildLocalTrajectory()
// while keeping the same input/output contract.
class RiskAwareLocalPlanner : public LocalPlannerInterface {
 public:
  RiskAwareLocalPlanner(const GridMap& map, const CollisionChecker& checker,
                        const FootprintModel& footprint);

  bool Initialize(ros::NodeHandle& pnh) override;

  void SetLocalObstacleMap(const GridMap* map, const CollisionChecker* checker);

  LocalPlanningResult Plan(const LocalPlannerState& current_state,
                           const RiskAwareGlobalReferenceData& global_ref) override;

 private:
  const GridMap& ActiveMap() const;
  const CollisionChecker& ActiveChecker() const;

  std::vector<GlobalReferencePointData> ExtractLocalReferenceWindow(
      const RiskAwareGlobalReferenceData& global_ref, int nearest_index,
      double* high_risk_ratio) const;

  LocalPlannerMode DetermineLocalPlannerMode(
      const std::vector<GlobalReferencePointData>& window,
      double high_risk_ratio,
      bool dynamic_obstacle_relevant) const;

  bool HasRelevantDynamicObstacle(
      const std::vector<GlobalReferencePointData>& window) const;

  bool HasDynamicObstacleNear(double wx, double wy, double radius) const;

  bool IsNewDynamicObstacleCell(int gx, int gy) const;

  bool BuildLocalTrajectory(const LocalPlannerState& current_state,
                            const std::vector<GlobalReferencePointData>& window,
                            LocalPlannerMode mode,
                            LocalPlanningResult* result) const;

  bool EvaluateLocalTrajectory(
      const std::vector<SE2State>& path,
      const std::vector<GlobalReferencePointData>& window,
      LocalPlanningResult* result) const;

  bool CheckNeedReplan(const LocalPlannerState& current_state,
                       const std::vector<GlobalReferencePointData>& window,
                       const LocalPlanningResult& partial) const;

  bool IsGoalReached(const LocalPlannerState& current_state,
                     const RiskAwareGlobalReferenceData& global_ref) const;

  int FindNearestReferenceIndex(const LocalPlannerState& current_state,
                                const RiskAwareGlobalReferenceData& global_ref) const;

  int FindNearestReferenceIndexInWindow(
      const SE2State& state,
      const std::vector<GlobalReferencePointData>& window,
      double* distance) const;

  std::vector<double> BuildSpeedSamples(const LocalPlannerState& current_state,
                                        const std::vector<GlobalReferencePointData>& window,
                                        LocalPlannerMode mode) const;

  std::vector<double> BuildCurvatureSamples(
      const LocalPlannerState& current_state,
      const std::vector<GlobalReferencePointData>& window) const;

  bool RolloutAckermannCandidate(
      const LocalPlannerState& current_state, double linear_velocity,
      double curvature, LocalPlannerMode mode,
      const std::vector<GlobalReferencePointData>& window,
      std::vector<SE2State>* path, double* cost,
      std::string* reject_reason) const;

  double PreferredSpeedForWindow(
      const std::vector<GlobalReferencePointData>& window,
      LocalPlannerMode mode) const;

  double SignedLateralError(const LocalPlannerState& current_state,
                            const GlobalReferencePointData& ref_point) const;

  double WrappedYawError(double yaw, double reference_yaw) const;

  double TargetClearance(LocalPlannerMode mode,
                         const GlobalReferencePointData& ref_point) const;

  void UpdateCommand(const LocalPlannerState& current_state,
                     const std::vector<GlobalReferencePointData>& window,
                     LocalPlanningResult* result) const;
  void ApplyCommandSmoothing(LocalPlanningResult* result) const;

  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
  const GridMap* local_obstacle_map_ = nullptr;
  const CollisionChecker* local_obstacle_checker_ = nullptr;
  const FootprintModel* footprint_ = nullptr;

  double window_length_ = 3.0;
  double window_resample_spacing_ = 0.25;
  double high_risk_ratio_threshold_ = 0.35;
  double low_risk_smoothing_alpha_ = 0.35;
  double high_risk_lateral_replan_threshold_ = 0.35;
  double high_risk_yaw_replan_threshold_ = 0.45;
  double strict_reference_deviation_threshold_ = 0.40;
  double low_risk_clearance_margin_ = 0.10;
  double high_risk_clearance_margin_ = 0.05;
  double clearance_target_cap_ = 0.35;
  double push_step_size_ = 0.05;
  double max_linear_speed_ = 0.35;
  double max_angular_speed_ = 0.8;
  double max_linear_accel_ = 0.8;
  double max_angular_accel_ = 0.8;
  double min_turn_linear_speed_ = 0.08;
  double min_turning_radius_ = 0.75;
  double angular_gain_ = 1.2;
  double sharp_turn_yaw_error_ = 0.65;
  double full_speed_yaw_error_ = 0.20;
  double pure_pursuit_lookahead_ = 0.80;
  double pure_pursuit_lateral_deadband_ = 0.03;
  double angular_deadband_ = 0.03;
  double command_smoothing_alpha_ = 0.35;
  double dynamic_obstacle_relevance_radius_ = 0.65;
  int push_max_iters_ = 8;
  int consecutive_failure_limit_ = 3;
  int blocked_window_limit_ = 2;
  double horizon_time_ = 2.5;
  double simulation_dt_ = 0.10;
  double control_period_ = 0.10;
  int velocity_samples_ = 5;
  int curvature_samples_ = 9;
  int target_curvature_samples_ = 6;
  bool allow_reverse_ = false;
  double goal_reached_xy_tolerance_ = 0.25;
  double goal_reached_yaw_tolerance_ = 0.35;
  double low_risk_reference_weight_ = 2.0;
  double high_risk_reference_weight_ = 8.0;
  double yaw_weight_ = 0.4;
  double obstacle_weight_ = 3.0;
  double progress_weight_ = 2.5;
  double speed_weight_ = 0.8;
  double curvature_weight_ = 0.15;
  double acceleration_weight_ = 0.8;
  double angular_acceleration_weight_ = 1.0;
  int consecutive_failures_ = 0;
  int consecutive_blocked_cycles_ = 0;
  bool use_dynamic_obstacles_for_cycle_ = false;
  LocalPlannerCommand last_command_;
  bool has_last_command_ = false;
};

}  // namespace isweep_planner
