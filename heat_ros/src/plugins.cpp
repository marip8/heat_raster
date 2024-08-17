#include <yaml-cpp/yaml.h>
#include <noether_gui/plugin_interface.h>
#include <heat_ros/heat_method_tool_path_planner_widget.h>

namespace heat_ros
{
struct HeatMethodToolPathPlannerWidgetPlugin : public noether::ToolPathPlannerWidgetPlugin
{
  QWidget* create(QWidget* parent = nullptr, const YAML::Node& config = {}) const override
  {
    auto widget = new HeatMethodToolPathPlannerWidget(parent);

    // Attempt to configure the widget
    if (!config.IsNull())
      widget->configure(config);

    return widget;
  }
};

} // namespace heat_ros

EXPORT_TPP_WIDGET_PLUGIN(heat_ros::HeatMethodToolPathPlannerWidgetPlugin, HeatMethodToolPathPlanner)
