#ifndef IIR_NAVIGATION__VISION_OBSTACLE_LAYER_HPP_
#define IIR_NAVIGATION__VISION_OBSTACLE_LAYER_HPP_

#include "nav2_costmap_2d/obstacle_layer.hpp"

namespace iir_navigation
{

class VisionObstacleLayer : public nav2_costmap_2d::ObstacleLayer
{
public:
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
};

}  // namespace iir_navigation
   // iir_navigation 命名空间结束。

#endif  // IIR_NAVIGATION__VISION_OBSTACLE_LAYER_HPP_
        // IIR_NAVIGATION__VISION_OBSTACLE_LAYER_HPP_ 头文件保护结束。
