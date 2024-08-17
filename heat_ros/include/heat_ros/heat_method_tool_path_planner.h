#pragma once

#include <noether_tpp/core/tool_path_planner.h>

namespace heat_ros
{
class HeatMethodToolPathPlanner : public noether::ToolPathPlanner
{
public:
  using noether::ToolPathPlanner::ToolPathPlanner;

  noether::ToolPaths plan(const pcl::PolygonMesh&) const override;
};

} // namespace heat_ros
