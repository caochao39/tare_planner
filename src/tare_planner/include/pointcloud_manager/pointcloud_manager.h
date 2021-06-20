/**
 * @file pointcloud_manager.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid of pointclouds
 * @version 0.1
 * @date 2019-12-08
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>

#include <Eigen/Core>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
// ROS
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include "grid/grid.h"
#include <utils/misc_utils.h>

namespace pointcloud_manager_ns
{
class PointCloudManager
{
public:
  typedef pcl::PointXYZRGBNormal PCLPointType;
  typedef pcl::PointCloud<pcl::PointXYZRGBNormal> PCLCloudType;
  typedef typename pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr PCLCloudTypePtr;

  std::vector<PCLCloudTypePtr> clouds_;

  explicit PointCloudManager(int row_num = 20, int col_num = 20, int max_cell_point_num = 100000, double cell_size = 24,
                             int neighbor_cell_num = 5);
  ~PointCloudManager() = default;
  void UpdateRobotPosition(const geometry_msgs::Point& robot_position);
  template <class InputPCLPointType>
  void UpdatePointCloud(const pcl::PointCloud<InputPCLPointType>& cloud_in)
  {
    for (const auto& cloud_in_point : cloud_in.points)
    {
      PCLPointType point;
      point.x = cloud_in_point.x;
      point.y = cloud_in_point.y;
      point.z = cloud_in_point.z;
      Eigen::Vector3i cell_sub = pointcloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
      if (!pointcloud_grid_->InRange(cell_sub))
        continue;
      int ind = pointcloud_grid_->Sub2Ind(cell_sub);
      pointcloud_grid_->GetCell(ind)->points.push_back(point);
    }

    for (int i = 0; i < pointcloud_grid_->GetCellNumber(); ++i)
    {
      if (pointcloud_grid_->GetCell(i)->points.empty())
        continue;
      cloud_dwz_filter_.setInputCloud(pointcloud_grid_->GetCell(i));
      cloud_dwz_filter_.setLeafSize(kCloudDwzFilterLeafSize, kCloudDwzFilterLeafSize, kCloudDwzFilterLeafSize);
      cloud_dwz_filter_.filter(*(pointcloud_grid_->GetCell(i)));
    }
  }

  void GetPointCloud(PCLCloudType& cloud_out);
  void GetMarker(visualization_msgs::Marker& marker);
  void GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud);
  Eigen::Vector3d GetNeighborCellsOrigin()
  {
    return neighbor_cells_origin_;
  }
  geometry_msgs::Point GetOrigin()
  {
    return origin_;
  }
  double& SetCloudDwzFilterLeafSize()
  {
    return kCloudDwzFilterLeafSize;
  }
  void GetCloudPointIndex(int index, int& cloud_index, int& cloud_point_index);
  int GetAllPointNum();

  void UpdateOldCloudPoints();
  void UpdateCoveredCloudPoints();
  void UpdateCoveredCloudPoints(int cloud_index, int point_index);

private:
  std::unique_ptr<grid_ns::Grid<PCLCloudTypePtr>> pointcloud_grid_;
  const int kRowNum;
  const int kColNum;
  const int kMaxCellPointNum;
  const double kCellSize;
  const int kNeighborCellNum;
  double kCloudDwzFilterLeafSize;

  geometry_msgs::Point robot_position_;
  geometry_msgs::Point origin_;
  Eigen::Vector3d neighbor_cells_origin_;

  int cur_row_idx_;
  int cur_col_idx_;

  bool initialized_;

  pcl::VoxelGrid<PCLPointType> cloud_dwz_filter_;

  std::vector<int> neighbor_indices_;

  void UpdateOrigin();
};
}  // namespace pointcloud_manager_ns