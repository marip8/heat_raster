#include <heat_ros/heat_method_tool_path_planner_widget.h>
#include <heat_ros/heat_method_tool_path_planner.h>

namespace heat_ros
{
noether::ToolPathPlanner::ConstPtr HeatMethodToolPathPlannerWidget::create() const
{
  return std::make_unique<HeatMethodToolPathPlanner>();
}

} // namespace heat_ros
