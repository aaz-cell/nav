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

  LocalPlanningResult Plan(const LocalPlannerState& current_state,
                           const RiskAwareGlobalReferenceData& global_ref) override;

 private:
  std::vector<GlobalReferencePointData> ExtractLocalReferenceWindow(
      const RiskAwareGlobalReferenceData& global_ref, int nearest_index,
      double* high_risk_ratio) const;

  LocalPlannerMode DetermineLocalPlannerMode(
      const std::vector<GlobalReferencePointData>& window,
      double high_risk_ratio) const;

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

  int FindNearestReferenceIndex(const LocalPlannerState& current_state,
                                const RiskAwareGlobalReferenceData& global_ref) const;

  double SignedLateralError(const LocalPlannerState& current_state,
                            const GlobalReferencePointData& ref_point) const;

  double WrappedYawError(double yaw, double reference_yaw) const;

  double TargetClearance(LocalPlannerMode mode,
                         const GlobalReferencePointData& ref_point) const;

  void UpdateCommand(const LocalPlannerState& current_state,
                     const std::vector<GlobalReferencePointData>& window,
                     LocalPlanningResult* result) const;

  const GridMap* map_ = nullptr;
  const CollisionChecker* checker_ = nullptr;
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
  double push_step_size_ = 0.05;
  double max_linear_speed_ = 0.35;
  double max_angular_speed_ = 0.8;
  double min_turn_linear_speed_ = 0.08;
  double min_turning_radius_ = 0.75;
  double angular_gain_ = 1.2;
  double sharp_turn_yaw_error_ = 0.65;
  double full_speed_yaw_error_ = 0.20;
  int push_max_iters_ = 8;
  int consecutive_failure_limit_ = 3;
  int blocked_window_limit_ = 2;
  int consecutive_failures_ = 0;
  int consecutive_blocked_cycles_ = 0;
};

}  // namespace isweep_planner
