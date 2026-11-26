#include "iir_navigation/vision_obstacle_layer.hpp"

#include <algorithm>
#include <mutex>

#include "pluginlib/class_list_macros.hpp"

namespace iir_navigation
{

void VisionObstacleLayer::updateBounds(
  double robot_x, double robot_y, double robot_yaw,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  std::lock_guard<nav2_costmap_2d::Costmap2D::mutex_t> guard(*getMutex());
  // Rebuild only the vision layer; other sensors keep their own observations.
  // 只重建视觉层，其他传感器保留各自的观测结果。
  const double previous_min_x = getOriginX();
  const double previous_min_y = getOriginY();
  const double previous_max_x = previous_min_x + getSizeInMetersX();
  const double previous_max_y = previous_min_y + getSizeInMetersY();
  resetMaps();
  ObstacleLayer::updateBounds(robot_x, robot_y, robot_yaw, min_x, min_y, max_x, max_y);
  *min_x = std::min({*min_x, previous_min_x, getOriginX()});
  *min_y = std::min({*min_y, previous_min_y, getOriginY()});
  *max_x = std::max({*max_x, previous_max_x, getOriginX() + getSizeInMetersX()});
  *max_y = std::max({*max_y, previous_max_y, getOriginY() + getSizeInMetersY()});
}

}  // namespace iir_navigation
   // iir_navigation 命名空间结束。

PLUGINLIB_EXPORT_CLASS(iir_navigation::VisionObstacleLayer, nav2_costmap_2d::Layer)
