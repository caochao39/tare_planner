#include <rclcpp/rclcpp.hpp>
#include "sensor_coverage_planner/sensor_coverage_planner_ground.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv, "tare_planner_node");
  rclcpp::Node::SharedPtr nh = rclcpp::Node::make_shared("tare_planner_node");

  sensor_coverage_planner_3d_ns::SensorCoveragePlanner3D tare_planner(nh, nh);

  ros::spin();
  return 0;
}