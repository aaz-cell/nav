#include "isweep_planner/framework/svsdf_runtime.h"

#include <algorithm>
#include <cmath>

namespace isweep_planner {

namespace {

bool SameReferencePointPosition(const GlobalReferencePointData& lhs,
                                const GlobalReferencePointData& rhs) {
  return std::abs(lhs.x - rhs.x) < 1e-6 && std::abs(lhs.y - rhs.y) < 1e-6;
}

// Append one sampled state into the paper-facing global reference.
//
// Field semantics:
// - x, y, yaw: geometric reference for downstream tracking.
// - risk_level: risk label inherited from MotionSegment.
// - clearance: local environment margin queried from the existing evaluator.
// - segment_id: segment index after risk segmentation.
// - s: accumulated arc length on the risk-aware reference.
// - preferred_speed: recommended local tracking speed derived from risk level.
void AppendReferenceSample(const SE2State& state, int segment_id,
                           RiskLevel risk, double clearance,
                           double preferred_speed,
                           RiskAwareGlobalReferenceData* reference) {
  if (reference == nullptr) {
    return;
  }

  GlobalReferencePointData point;
  point.x = state.x;
  point.y = state.y;
  point.yaw = state.yaw;
  point.risk_level = risk == RiskLevel::HIGH ? 1 : 0;
  point.clearance = clearance;
  point.segment_id = segment_id;
  point.preferred_speed = preferred_speed;

  if (!reference->points.empty()) {
    const GlobalReferencePointData& previous = reference->points.back();
    if (SameReferencePointPosition(previous, point)) {
      point.s = previous.s;
      reference->points.back() = point;
      return;
    }

    const double dx = point.x - previous.x;
    const double dy = point.y - previous.y;
    point.s = previous.s + std::sqrt(dx * dx + dy * dy);
  }

  reference->points.push_back(point);
}

}  // namespace

RiskAwareGlobalReferenceData SvsdfRuntime::BuildRiskAwareGlobalReference(
    const CandidateResult& candidate) const {
  RiskAwareGlobalReferenceData reference;
  reference.stamp = ros::Time::now();
  reference.frame_id =
      latest_map_.header.frame_id.empty() ? "map" : latest_map_.header.frame_id;
  reference.num_segments = static_cast<int>(candidate.segments.size());
  reference.num_high_risk_segments = candidate.high_risk_segments;

  if (candidate.segment_trajectories.empty() || candidate.segments.empty()) {
    return reference;
  }

  const size_t segment_count =
      std::min(candidate.segments.size(), candidate.segment_trajectories.size());
  for (size_t i = 0; i < segment_count; ++i) {
    const Trajectory& segment_traj = candidate.segment_trajectories[i];
    if (segment_traj.empty()) {
      continue;
    }

    const RiskLevel risk = candidate.segments[i].risk;
    const double preferred_speed = PreferredSpeedForRisk(risk);
    const double duration = segment_traj.totalDuration();
    const double sample_dt = std::max(0.02, reference_sample_dt_);

    // Data source for the paper's "risk-aware global reference trajectory":
    // - geometry/yaw come from the stitched segment trajectories selected by the
    //   candidate evaluation stage.
    // - risk labels come from MotionSegment::risk assigned by SE2SequenceGenerator.
    // - clearance is queried with the existing SVSDF evaluator, which preserves
    //   one consistent notion of safety margin before handing the reference to
    //   the local planner.
    // - preferred speed is a compact semantic cue telling the local planner how
    //   aggressively to track the current segment.
    for (double t = 0.0; t < duration; t += sample_dt) {
      const SE2State state = segment_traj.sample(t);
      AppendReferenceSample(state, static_cast<int>(i), risk,
                            svsdf_evaluator_.evaluate(state), preferred_speed,
                            &reference);
    }

    const SE2State final_state = segment_traj.sample(duration);
    AppendReferenceSample(final_state, static_cast<int>(i), risk,
                          svsdf_evaluator_.evaluate(final_state), preferred_speed,
                          &reference);
  }

  if (!reference.points.empty()) {
    reference.total_length = reference.points.back().s;
  }
  return reference;
}

}  // namespace isweep_planner
