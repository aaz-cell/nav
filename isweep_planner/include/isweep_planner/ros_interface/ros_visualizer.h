#pragma once

#include <vector>
#include <nav_msgs/Path.h>
#include <visualization_msgs/MarkerArray.h>
#include "isweep_planner/core/common.h"
#include "isweep_planner/env/footprint_model.h"
#include "isweep_planner/framework/svsdf_runtime.h"
#include "isweep_planner/local_planner/local_planner_types.h"

namespace isweep_planner {

class RosVisualizer {
public:
  static nav_msgs::Path MakePath(const std::vector<SE2State>& states);
  static nav_msgs::Path MakePathFromTopo(const TopoPath& path_in);
  static nav_msgs::Path MakeTrajectoryPath(const Trajectory& traj);
  
  static visualization_msgs::MarkerArray MakeFootprintMarkers(
      const Trajectory& traj, 
      const FootprintModel& footprint, 
      const std::string& frame_id = "map",
      double dt = 0.5);

  static visualization_msgs::MarkerArray MakeRiskAwareReferenceMarkers(
      const RiskAwareGlobalReferenceData& reference,
      const std::string& frame_id = "map");

  static visualization_msgs::MarkerArray MakeLocalPlannerMarkers(
      const LocalPlanningResult& result,
      const std::string& frame_id = "map");
};

}  // namespace isweep_planner
