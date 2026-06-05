#pragma once

#include <string>
#include <vector>

#include "isweep_planner/core/common.h"
#include "isweep_planner/framework/svsdf_runtime.h"

namespace isweep_planner {

struct LocalPlannerState {
  double x = 0.0;
  double y = 0.0;
  double yaw = 0.0;
  double v = 0.0;
  bool has_velocity = false;
};

struct LocalPlannerCommand {
  double linear_velocity = 0.0;
  double angular_velocity = 0.0;
};

enum class LocalPlannerMode {
  LOW_RISK_FOLLOW = 0,
  HIGH_RISK_STRICT = 1,
  BLOCKED_RECOVERY = 2,
};

enum class LocalPlannerStatus {
  OK = 0,
  BLOCKED = 1,
  DIVERGED = 2,
  NEED_REPLAN = 3,
  NO_REFERENCE = 4,
};

struct LocalPlanningResult {
  bool success = false;
  bool has_command = false;
  bool need_replan = false;
  bool blocked = false;
  LocalPlannerMode mode = LocalPlannerMode::LOW_RISK_FOLLOW;
  LocalPlannerStatus status = LocalPlannerStatus::NO_REFERENCE;
  std::vector<GlobalReferencePointData> local_reference;
  std::vector<SE2State> local_path;
  LocalPlannerCommand local_cmd;
  double tracking_error = 0.0;
  double yaw_error = 0.0;
  double min_clearance = kInf;
  int matched_index = -1;
  int consecutive_failures = 0;
  std::string debug_reason;
};

const char* ToString(LocalPlannerMode mode);
const char* ToString(LocalPlannerStatus status);

}  // namespace isweep_planner
