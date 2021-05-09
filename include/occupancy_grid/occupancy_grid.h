/**
 * @file occupancy_grid.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements an occupancy grid
 * @version 0.1
 * @date 2020-03-18
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <grid/grid.h>
#include <geometry_msgs/Point.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <utils/misc_utils.h>

namespace occupancy_grid_ns
{
class OccupancyGrid
{
public:
  enum CellState : char
  {
    UNKNOWN = 0,
    OCCUPIED = 1,
    FREE = 2,
    NOT_FRONTIER = 3
  };
  explicit OccupancyGrid(const Eigen::Vector3d& origin, const Eigen::Vector3d& range,
                         const Eigen::Vector3d& resolution);
  explicit OccupancyGrid(double cell_size_x = 1.0, double cell_size_y = 1.0, double cell_size_z = 1.0);
  ~OccupancyGrid() = default;

  void SetOrigin(const Eigen::Vector3d& origin)
  {
    origin_ = origin;
    grid_array_->SetOrigin(origin);
  }
  void SetEliminateFrontier(bool eliminate)
  {
    eliminate_frontier_ = eliminate;
  }
  void SetEliminateFrontierOrigin(const Eigen::Vector3d& eliminate_frontier_origin)
  {
    eliminate_frontier_origin_ = eliminate_frontier_origin;
  }
  void SetResolution(const Eigen::Vector3d& resolution)
  {
    resolution_ = resolution;
    grid_array_->SetResolution(resolution);
  }
  void SetExtractFrontierRange(const Eigen::Vector3d& range)
  {
    range_ = range;
  }

  Eigen::Vector3i PointToSub(double x, double y, double z);

  void GetVisualizationCloudInRange(const Eigen::Vector3d& origin, const Eigen::Vector3d& range,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& occupied_cloud,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& free_cloud,
                                    pcl::PointCloud<pcl::PointXYZI>::Ptr& unknown_cloud);
  void RayTrace(const Eigen::Vector3d& origin, const Eigen::Vector3d& range);
  void RayTrace(const Eigen::Vector3d& origin);
  void RayTraceHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                      std::vector<Eigen::Vector3i>& cells);
  bool CanRayTrace(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub);

  template <class PointType>
  void UpdateOccupancy(typename pcl::PointCloud<PointType>::Ptr& cloud)
  {
    updated_grid_indices_.clear();
    for (const auto& point : cloud->points)
    {
      Eigen::Vector3i sub = grid_array_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
      if (grid_array_->InRange(sub))
      {
        grid_array_->SetCellValue(sub, OCCUPIED);
        int ind = grid_array_->Sub2Ind(sub);
        updated_grid_indices_.push_back(ind);
      }
    }
  }

  void GetFrontierInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr& frontier_cloud, const Eigen::Vector3d& origin,
                          const Eigen::Vector3d& range);
  void GetFrontierInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr& frontier_cloud, const Eigen::Vector3d& origin);

private:
  std::unique_ptr<grid_ns::Grid<CellState>> grid_array_;
  std::vector<int> updated_grid_indices_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d size_;
  Eigen::Vector3d range_;
  Eigen::Vector3d resolution_;
  Eigen::Vector3d eliminate_frontier_origin_;
  bool eliminate_frontier_;
};
}  // namespace occupancy_grid_ns