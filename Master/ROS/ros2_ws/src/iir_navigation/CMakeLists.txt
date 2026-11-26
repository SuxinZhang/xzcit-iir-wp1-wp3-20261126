cmake_minimum_required(VERSION 3.8)
project(iir_navigation)

find_package(ament_cmake REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)

add_library(vision_obstacle_layer SHARED src/vision_obstacle_layer.cpp)
target_compile_features(vision_obstacle_layer PUBLIC cxx_std_17)
target_include_directories(vision_obstacle_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(vision_obstacle_layer nav2_costmap_2d pluginlib)
pluginlib_export_plugin_description_file(nav2_costmap_2d costmap_plugins.xml)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(lifecycle_msgs REQUIRED)
  add_executable(vision_layer_probe test/vision_layer_probe.cpp)
  ament_target_dependencies(vision_layer_probe lifecycle_msgs nav2_costmap_2d)
  install(TARGETS vision_layer_probe DESTINATION bin)
  ament_add_pytest_test(test_costmap_sensors test/test_costmap_sensors.py TIMEOUT 60)
endif()

install(TARGETS vision_obstacle_layer LIBRARY DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
ament_package()
