#pragma once

#include <noether_tpp/core/tool_path_planner.h>
#include <noether_gui/widgets.h>

namespace heat_ros
{
class HeatMethodToolPathPlannerWidget : public noether::ToolPathPlannerWidget
{
public:
  using noether::ToolPathPlannerWidget::ToolPathPlannerWidget;

  typename noether::ToolPathPlanner::ConstPtr create() const override;
};

} // namespace heat_ros
