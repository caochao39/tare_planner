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
TAREVisualizer::TAREVisualizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
{
  ReadParameters(nh_private);

  marker_publisher_ = nh.advertise<visualization_msgs::Marker>("tare_visualizer/marker", 1);
  local_path_publisher_ = nh.advertise<nav_msgs::Path>("tare_visualizer/local_path", 1);

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
bool TAREVisualizer::ReadParameters(ros::NodeHandle& nh)
{
  kExploringSubspaceMarkerColorGradientAlpha =
      misc_utils_ns::getParam<bool>(nh, "kExploringSubspaceMarkerColorGradientAlpha", true);
  kExploringSubspaceMarkerColorMaxAlpha =
      misc_utils_ns::getParam<double>(nh, "kExploringSubspaceMarkerColorMaxAlpha", 1.0);
  kExploringSubspaceMarkerColor.r = misc_utils_ns::getParam<double>(nh, "kExploringSubspaceMarkerColorR", 0.0);
  kExploringSubspaceMarkerColor.g = misc_utils_ns::getParam<double>(nh, "kExploringSubspaceMarkerColorG", 1.0);
  kExploringSubspaceMarkerColor.b = misc_utils_ns::getParam<double>(nh, "kExploringSubspaceMarkerColorB", 0.0);
  kExploringSubspaceMarkerColor.a = misc_utils_ns::getParam<double>(nh, "kExploringSubspaceMarkerColorA", 1.0);

  kLocalPlanningHorizonMarkerColor.r = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonMarkerColorR", 0.0);
  kLocalPlanningHorizonMarkerColor.g = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonMarkerColorG", 1.0);
  kLocalPlanningHorizonMarkerColor.b = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonMarkerColorB", 0.0);
  kLocalPlanningHorizonMarkerColor.a = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonMarkerColorA", 1.0);

  kLocalPlanningHorizonMarkerWidth = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonMarkerWidth", 0.3);
  kGlobalSubspaceSize = misc_utils_ns::getParam<double>(nh, "kGridWorldCellSize", 7.7);
  kGlobalSubspaceHeight = misc_utils_ns::getParam<double>(nh, "kGridWorldCellHeight", 3.0);

  double viewpoint_num_x = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/number_x", 35);
  double viewpoint_num_y = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/number_y", 35);
  double viewpoint_resolution_x = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_x", 1.1);
  double viewpoint_resolution_y = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resolution_y", 1.1);
  kLocalPlanningHorizonSizeX = viewpoint_num_x * viewpoint_resolution_x;
  kLocalPlanningHorizonSizeY = viewpoint_num_y * viewpoint_resolution_y;
  kLocalPlanningHorizonSizeZ = misc_utils_ns::getParam<double>(nh, "kLocalPlanningHorizonHeight", 3.0);
  return true;
}

void TAREVisualizer::InitializeMarkers()
{
  global_subspaces_marker_->SetType(visualization_msgs::Marker::CUBE_LIST);
  global_subspaces_marker_->SetScale(kGlobalSubspaceSize, kGlobalSubspaceSize, kGlobalSubspaceHeight);
  global_subspaces_marker_->SetColorRGBA(kExploringSubspaceMarkerColor);

  local_planning_horizon_marker_->SetType(visualization_msgs::Marker::LINE_LIST);
  local_planning_horizon_marker_->SetScale(kLocalPlanningHorizonMarkerWidth, 0, 0);
  local_planning_horizon_marker_->SetColorRGBA(kExploringSubspaceMarkerColor);
}

void TAREVisualizer::GetLocalPlanningHorizonMarker(double x, double y, double z)
{
  local_planning_horizon_origin_.x = x;
  local_planning_horizon_origin_.y = y;
  local_planning_horizon_origin_.z = z - kLocalPlanningHorizonSizeZ / 2;

  geometry_msgs::Point upper_right;
  upper_right.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_right.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  upper_right.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point lower_right;
  lower_right.x = local_planning_horizon_origin_.x;
  lower_right.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  lower_right.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point upper_left;
  upper_left.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_left.y = local_planning_horizon_origin_.y;
  upper_left.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point lower_left;
  lower_left.x = local_planning_horizon_origin_.x;
  lower_left.y = local_planning_horizon_origin_.y;
  lower_left.z = local_planning_horizon_origin_.z + kLocalPlanningHorizonSizeZ;

  geometry_msgs::Point upper_right2;
  upper_right2.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_right2.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  upper_right2.z = local_planning_horizon_origin_.z;

  geometry_msgs::Point lower_right2;
  lower_right2.x = local_planning_horizon_origin_.x;
  lower_right2.y = local_planning_horizon_origin_.y + kLocalPlanningHorizonSizeY;
  lower_right2.z = local_planning_horizon_origin_.z;

  geometry_msgs::Point upper_left2;
  upper_left2.x = local_planning_horizon_origin_.x + kLocalPlanningHorizonSizeX;
  upper_left2.y = local_planning_horizon_origin_.y;
  upper_left2.z = local_planning_horizon_origin_.z;

  geometry_msgs::Point lower_left2;
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

void TAREVisualizer::GetGlobalSubspaceMarker(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world,
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
    geometry_msgs::Point cell_center = grid_world->GetCellPosition(cell_ind);
    std_msgs::ColorRGBA color;
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
    global_subspaces_marker_->SetAction(visualization_msgs::Marker::ADD);
    global_subspaces_marker_->Publish();
  }
  else
  {
    global_subspaces_marker_->SetAction(visualization_msgs::Marker::DELETE);
    global_subspaces_marker_->Publish();
  }
}

}  // namespace tare_visualizer_ns
