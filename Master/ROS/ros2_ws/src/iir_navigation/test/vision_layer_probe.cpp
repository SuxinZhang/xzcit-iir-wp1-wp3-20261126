#include <chrono>
#include <iostream>
#include <memory>
#include <thread>

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  if (argc != 2) {
    return 2;
  }
  const char * ros_args[] = {argv[0], "--ros-args", "--params-file", argv[1]};
  rclcpp::init(4, ros_args);
  auto costmap = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap");
  if (costmap->configure().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    std::cerr << "Costmap configure failed" << std::endl;
    return 1;
  }
  auto * layers = costmap->getLayeredCostmap();
  layers->updateMap(0.0, 0.0, 0.0);
  nav2_costmap_2d::Layer * vision = nullptr;
  for (const auto & layer : *layers->getPlugins()) {
    if (layer->getName() == "vision_layer") {
      vision = layer.get();
    }
  }
  if (!vision || !vision->isCurrent()) {
    std::cerr << "Vision layer was not initially current" << std::endl;
    return 1;
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(1250));
  layers->updateMap(0.0, 0.0, 0.0);
  const bool current = vision->isCurrent();
  costmap.reset();
  rclcpp::shutdown();
  if (current) {
    std::cerr << "Costmap remained current without vision data" << std::endl;
    return 1;
  }
  return 0;
}
