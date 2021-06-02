/**
 * @file visualization.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that visualizes the planning process
 * @version 0.1
 * @date 2021-06-01
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "utils/pointcloud_utils.h"
#include "grid_world/grid_world.h"
#include "utils/misc_utils.h"

namespace tare_visualizer_ns
{
class TAREVisualizer
{
public:
  explicit TAREVisualizer(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  bool ReadParameters(ros::NodeHandle& nh);

  void InitializeMarkers();
  void GetLocalPlanningHorizonMarker(double x, double y, double z);
  void GetGlobalSubspaceMarker(const std::unique_ptr<grid_world_ns::GridWorld>& grid_world,
                               const std::vector<int>& ordered_cell_indices);
  void PublishMarkers();

private:
  const std::string kWorldFrameID = "map";
  bool kExploringSubspaceMarkerColorGradientAlpha;
  double kExploringSubspaceMarkerColorMaxAlpha;
  std_msgs::ColorRGBA kExploringSubspaceMarkerColor;
  std_msgs::ColorRGBA kLocalPlanningHorizonMarkerColor;
  double kLocalPlanningHorizonMarkerWidth;
  double kLocalPlanningHorizonSizeX;
  double kLocalPlanningHorizonSizeY;
  double kLocalPlanningHorizonSizeZ;
  double kGlobalSubspaceSize;
  double kGlobalSubspaceHeight;

  ros::Publisher marker_publisher_;
  ros::Publisher uncovered_surface_point_publisher_;
  ros::Publisher viewpoint_candidate_publisher_;
  ros::Publisher viewpoint_publisher_;
  ros::Publisher local_path_publisher_;
  ros::Publisher global_path_publisher_;

  misc_utils_ns::Marker::Ptr global_subspaces_marker_;
  misc_utils_ns::Marker::Ptr local_planning_horizon_marker_;
  pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>::Ptr uncovered_surface_point_cloud_;
  pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>::Ptr viewpoint_candidate_cloud_;
  pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>::Ptr viewpoint_cloud_;
  nav_msgs::Path local_path_;
  nav_msgs::Path global_path_;

  geometry_msgs::Point local_planning_horizon_origin_;
  geometry_msgs::Point global_subspace_origin_;
  geometry_msgs::Point global_subspace_size_;
};
}  // namespace tare_visualizer_ns