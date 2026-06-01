#pragma once

#include <ros/ros.h>

#include "isweep_planner/local_planner/local_planner_types.h"

namespace isweep_planner {

class LocalPlannerInterface {
 public:
  virtual ~LocalPlannerInterface() = default;

  virtual bool Initialize(ros::NodeHandle& pnh) = 0;

  virtual LocalPlanningResult Plan(
      const LocalPlannerState& current_state,
      const RiskAwareGlobalReferenceData& global_ref) = 0;
};

}  // namespace isweep_planner
