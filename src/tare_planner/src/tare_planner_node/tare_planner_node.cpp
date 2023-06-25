#include <rclcpp/rclcpp.hpp>
#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D>();
  node->initialize();
  std::cout << "finished initializing node, spinning now" << std::endl;
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}