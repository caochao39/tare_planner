/**
 * @file rolling_occupancy_grid.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a rolling occupancy grid
 * @version 0.1
 * @date 2021-06-16
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "grid/grid.h"
#include "rolling_grid/rolling_grid.h"
#include "utils/misc_utils.h"

namespace rolling_occupancy_grid_ns
{
class RollingOccupancyGrid
{
public:
  enum CellState : char
  {
    UNKNOWN = 0,
    OCCUPIED = 1,
    FREE = 2,
    NOT_FRONTIER = 3
  };

  explicit RollingOccupancyGrid(rclcpp::Node::SharedPtr nh);
  ~RollingOccupancyGrid() = default;

  Eigen::Vector3d GetResolution()
  {
    return resolution_;
  }

  void InitializeOrigin(const Eigen::Vector3d& origin);
  bool UpdateRobotPosition(const Eigen::Vector3d& robot_position);
  template <class PointType>
  void UpdateOccupancy(typename pcl::PointCloud<PointType>::Ptr& cloud)
  {
    if (!initialized_)
    {
      return;
    }
    updated_grid_indices_.clear();
    for (const auto& point : cloud->points)
    {
      Eigen::Vector3i sub = occupancy_array_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
      if (occupancy_array_->InRange(sub))
      {
        int ind = occupancy_array_->Sub2Ind(sub);
        int array_ind = rolling_grid_->GetArrayInd(ind);
        occupancy_array_->SetCellValue(array_ind, OCCUPIED);
        updated_grid_indices_.push_back(ind);
      }
    }
  }
  void UpdateOccupancyStatus(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void RayTrace(const Eigen::Vector3d& origin, const Eigen::Vector3d& range);
  void RayTrace(const Eigen::Vector3d& origin);
  void RayTraceHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                      std::vector<Eigen::Vector3i>& cells);

  void GetFrontier(pcl::PointCloud<pcl::PointXYZI>::Ptr& frontier_cloud, const Eigen::Vector3d& origin,
                   const Eigen::Vector3d& range);
  pcl::PointCloud<pcl::PointXYZI>::Ptr GetRolledOutOccupancyCloud()
  {
    return occupancy_cloud_;
  }
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud);

private:
  bool initialized_;
  int dimension_;
  Eigen::Vector3d range_;
  Eigen::Vector3i grid_size_;
  Eigen::Vector3d rollover_range_;
  Eigen::Vector3i rollover_step_size_;
  Eigen::Vector3d resolution_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d robot_position_;
  std::unique_ptr<rolling_grid_ns::RollingGrid> rolling_grid_;
  std::unique_ptr<grid_ns::Grid<CellState>> occupancy_array_;
  std::vector<int> updated_grid_indices_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr occupancy_cloud_;

  bool InRange(const Eigen::Vector3i& sub, const Eigen::Vector3i& sub_min, const Eigen::Vector3i& sub_max);

  // void InitializeOrigin();
};
}  // namespace rolling_occupancy_grid_ns
