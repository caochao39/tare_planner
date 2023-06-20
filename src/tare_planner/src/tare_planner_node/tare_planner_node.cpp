#include <rclcpp/rclcpp.hpp>
#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D>());
  rclcpp::shutdown();
  return 0;
}