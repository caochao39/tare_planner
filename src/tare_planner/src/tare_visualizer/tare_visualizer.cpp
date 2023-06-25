/**
 * @file tare_visualizer.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that visualizes the planning process
 * @version 0.1
 * @date 2021-06-01
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "tare_visualizer/tare_visualizer.h"

namespace tare_visualizer_ns
{
TAREVisualizer::TAREVisualizer(rclcpp::Node::SharedPtr nh)
{
  ReadParameters(nh);

  marker_publisher_ = nh->create_publisher<visualization_msgs::msg::Marker>("tare_visualizer/marker", 1);
  local_path_publisher_ = nh->create_publisher<nav_msgs::msg::Path>("tare_visualizer/local_path", 1);

  global_subspaces_marker_ =
      std::make_shared<misc_utils_ns::Marker>(nh, "tare_visualizer/exploring_subspaces", kWorldFrameID);
  local_planning_horizon_marker_ =
      std::make_shared<misc_utils_ns::Marker>(nh, "tare_visualizer/local_planning_horizon", kWorldFrameID);

  uncovered_surface_point_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "tare_visualizer/uncovered_surface_points", kWorldFrameID);
  viewpoint_candidate_cloud_ = std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(
      nh, "tare_visualizer/viewpoint_candidates", kWorldFrameID);
  viewpoint_cloud_ =
      std::make_shared<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "tare_visualizer/viewpoints", kWorldFrameID);

  InitializeMarkers();
}
bool TAREVisualizer::ReadParameters(rclcpp::Node::SharedPtr nh)
{
  nh->get_parameter("kExploringSubspaceMarkerColorGradientAlpha", kExploringSubspaceMarkerColorGradientAlpha);
  nh->get_parameter("kExploringSubspaceMarkerColorMaxAlpha", kExploringSubspaceMarkerColorMaxAlpha);
  kExploringSubspaceMarkerColor.r = nh->get_parameter("kExploringSubspaceMarkerColorR").as_double();
  kExploringSubspaceMarkerColor.g = nh->get_parameter("kExploringSubspaceMarkerColorG").as_double();
  kExploringSubspaceMarkerColor.b = nh->get_parameter("kExploringSubspaceMarkerColorB").as_double();
  kExploringSubspaceMarkerColor.a = nh->get_parameter("kExploringSubspaceMarkerColorA").as_double();
  kLocalPlanningHorizonMarkerColor.r = nh->get_parameter("kLocalPlanningHorizonMarkerColorR").as_double();
  kLocalPlanningHorizonMarkerColor.g = nh->get_parameter("kLocalPlanningHorizonMarkerColorG").as_double();
  kLocalPlanningHorizonMarkerColor.b = nh->get_parameter("kLocalPlanningHorizonMarkerColorB").as_double();
  kLocalPlanningHorizonMarkerColor.a = nh->get_parameter("kLocalPlanningHorizonMarkerColorA").as_double();
  nh->get_parameter("kLocalPlanningHorizonMarkerColorA", kLocalPlanningHorizonMarkerWidth);
  int viewpoint_num_x = nh->get_parameter("viewpoint_manager/number_x").as_int();
  int viewpoint_num_y = nh->get_parameter("viewpoint_manager/number_y").as_int();
  double viewpoint_resolution_x = nh->get_parameter("viewpoint_manager/resolution_x").as_double();
  double viewpoint_resolution_y = nh->get_parameter("viewpoint_manager/resolution_x").as_double();
  kGlobalSubspaceSize = viewpoint_num_x * viewpoint_resolution_x / 5;
  kLocalPlanningHorizonSizeX = viewpoint_num_x * viewpoint_resolution_x;
  kLocalPlanningHorizonSizeY = viewpoint_num_y * viewpoint_resolution_y;
  nh->get_parameter("kGridWorldCellHeight", kGlobalSubspaceHeight);
  nh->get_parameter("kLocalPlanningHorizonHeight", kLocalPlanningHorizonSizeZ);

  return true;
}

void TAREVisualizer::InitializeMarkers()
{
  global_subspaces_marker_->SetType(visualization_msgs::msg::Marker::CUBE_LIST);
  global_subspaces_marker_->SetScale(kGlobalSubspaceSize, kGlobalSubspaceSize, kGlobalSubspaceHeight);
  global_subspaces_marker_->SetColorRGBA(kExploringSubspaceMarkerColor);

  local_planning_horizon_marker_->SetType(visualization_msgs::msg::Marker::LINE_LIST);
  local_planning_horizon_marker_->SetScale(kLocalPlanningHorizonMarkerWidth, 0, 0);
  local_planning_horizon_marker_->SetColorRGBA(kExploringSubspaceMarkerColor);
}

void TAREVisualizer::GetLocalPlanningHorizonMarker(double x, double y, double z)
{
  local_planning_horizon_origin_.x = x;
  local_planning_horizon_origin_.y = y;
  local_planning_horizon_origin_.z = z - kLocalPlanningHorizonSizeZ / 2;

  geometry_msgs::msg::Point upper_right;
  upper_right.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_right.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  upper_right.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::msg::Point lower_right;
  lower_right.x = local_planning_horizon_origin_.x;
  lower_right.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  lower_right.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::msg::Point upper_left;
  upper_left.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_left.y = local_planning_horizon_origin_.y;
  upper_left.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::msg::Point lower_left;
  lower_left.x = local_planning_horizon_origin_.x;
  lower_left.y = local_planning_horizon_origin_.y;
  lower_left.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::msg::Point upper_right2;
  upper_right2.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_right2.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  upper_right2.z = local_planning_horizon_origin_.z;

  geometry_msgs::msg::Point lower_right2;
  lower_right2.x = local_planning_horizon_origin_.x;
  lower_right2.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  lower_right2.z = local_planning_horizon_origin_.z;

  geometry_msgs::msg::Point upper_left2;
  upper_left2.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_left2.y = local_planning_horizon_origin_.y;
  upper_left2.z = local_planning_horizon_origin_.z;

  geometry_msgs::msg::Point lower_left2;
  lower_left2.x = local_planning_horizon_origin_.x;
  lower_left2.y = local_planning_horizon_origin_.y;
  lower_left2.z = local_planning_horizon_origin_.z;

  local_planning_horizon_marker_->marker_.points.clear();

  local_planning_horizon_marker_->marker_.points.push_back(upper_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_right);

  local_planning_horizon_marker_->marker_.points.push_back(upper_right);
  local_planning_horizon_marker_->marker_.points.push_back(upper_right2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right2);

  local_planning_horizon_marker_->marker_.points.push_back(upper_right2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_right2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left2);
  local_planning_horizon_marker_->marker_.points.push_back(lower_left2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_left2);
  local_planning_horizon_marker_->marker_.points.push_back(upper_right2);
}

void TAREVisualizer::GetGlobalSubspaceMarker(const std::shared_ptr<grid_world_ns::GridWorld>& grid_world,
                                             const std::vector<int>& ordered_cell_indices)
{
  global_subspaces_marker_->marker_.points.clear();
  global_subspaces_marker_->marker_.colors.clear();
  int cell_num = ordered_cell_indices.size();
  for (int i = 0; i < cell_num; i++)
  {
    int cell_ind = ordered_cell_indices[i];
    if (!grid_world->IndInBound(cell_ind))
    {
      continue;
    }
    geometry_msgs::msg::Point cell_center = grid_world->GetCellPosition(cell_ind);
    std_msgs::msg::ColorRGBA color;
    color.r = 0.0;
    color.g = 1.0;
    color.b = 0.0;
    if (kExploringSubspaceMarkerColorGradientAlpha)
    {
      color.a = ((cell_num - i) * 1.0 / cell_num) * kExploringSubspaceMarkerColorMaxAlpha;
    }
    else
    {
      color.a = 1.0;
    }
    global_subspaces_marker_->marker_.points.push_back(cell_center);
    global_subspaces_marker_->marker_.colors.push_back(color);
  }
}

void TAREVisualizer::PublishMarkers()
{
  local_planning_horizon_marker_->Publish();
  if (!global_subspaces_marker_->marker_.points.empty())
  {
    global_subspaces_marker_->SetAction(visualization_msgs::msg::Marker::ADD);
    global_subspaces_marker_->Publish();
  }
  else
  {
    global_subspaces_marker_->SetAction(visualization_msgs::msg::Marker::DELETE);
    global_subspaces_marker_->Publish();
  }
}

}  // namespace tare_visualizer_ns
