/**
 * @file occupancy_grid.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a 3D grid
 * @version 0.1
 * @date 2020-03-18
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "occupancy_grid/occupancy_grid.h"

namespace occupancy_grid_ns
{
OccupancyGrid::OccupancyGrid(const Eigen::Vector3d& origin, const Eigen::Vector3d& size,
                             const Eigen::Vector3d& resolution)
  : origin_(origin)
  , size_(size)
  , resolution_(resolution)
  , eliminate_frontier_(false)
  , eliminate_frontier_origin_(Eigen::Vector3d::Zero())
{
  Eigen::Vector3i cell_number;
  for (int i = 0; i < 3; i++)
  {
    cell_number(i) = static_cast<int>(size_(i) / resolution_(i));
  }
  grid_array_ = std::make_unique<grid_ns::Grid<CellState>>(cell_number, UNKNOWN, origin_, resolution_);
}

void OccupancyGrid::RayTraceHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                   std::vector<Eigen::Vector3i>& cells)
{
  cells.clear();
  MY_ASSERT(grid_array_->InRange(start_sub));
  MY_ASSERT(grid_array_->InRange(end_sub));

  if (start_sub == end_sub)
  {
    cells.push_back(start_sub);
    return;
  }
  Eigen::Vector3i diff_sub = end_sub - start_sub;
  double max_dist = diff_sub.squaredNorm();
  int step_x = misc_utils_ns::signum(diff_sub.x());
  int step_y = misc_utils_ns::signum(diff_sub.y());
  int step_z = misc_utils_ns::signum(diff_sub.z());
  double t_max_x = step_x == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.x(), diff_sub.x());
  double t_max_y = step_y == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.y(), diff_sub.y());
  double t_max_z = step_z == 0 ? DBL_MAX : misc_utils_ns::intbound(start_sub.z(), diff_sub.z());
  double t_delta_x = step_x == 0 ? DBL_MAX : (double)step_x / (double)diff_sub.x();
  double t_delta_y = step_y == 0 ? DBL_MAX : (double)step_y / (double)diff_sub.y();
  double t_delta_z = step_z == 0 ? DBL_MAX : (double)step_z / (double)diff_sub.z();
  double dist = 0;
  Eigen::Vector3i cur_sub = start_sub;

  while (true)
  {
    MY_ASSERT(grid_array_->InRange(cur_sub));
    cells.push_back(cur_sub);
    dist = (cur_sub - start_sub).squaredNorm();
    if (cur_sub == end_sub || dist > max_dist || grid_array_->GetCellValue(cur_sub) == OCCUPIED)
    {
      return;
    }
    if (t_max_x < t_max_y)
    {
      if (t_max_x < t_max_z)
      {
        cur_sub.x() += step_x;
        t_max_x += t_delta_x;
      }
      else
      {
        cur_sub.z() += step_z;
        t_max_z += t_delta_z;
      }
    }
    else
    {
      if (t_max_y < t_max_z)
      {
        cur_sub.y() += step_y;
        t_max_y += t_delta_y;
      }
      else
      {
        cur_sub.z() += step_z;
        t_max_z += t_delta_z;
      }
    }
  }
}

bool OccupancyGrid::CanRayTrace(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub)
{
  std::vector<Eigen::Vector3i> cells;
  RayTraceHelper(start_sub, end_sub, cells);
  if (cells.empty())
  {
    return false;
  }
  else
  {
    if (cells.front() == start_sub && cells.back() == end_sub)
    {
      return true;
    }
  }
}

void OccupancyGrid::RayTrace(const Eigen::Vector3d& origin, const Eigen::Vector3d& range)
{
  Eigen::Vector3i sub_max = grid_array_->Pos2Sub(origin + range);
  Eigen::Vector3i sub_min = grid_array_->Pos2Sub(origin - range);
  Eigen::Vector3i origin_sub = grid_array_->Pos2Sub(origin);
  int ray_trace_count = 0;
  if (!grid_array_->InRange(origin_sub))
  {
    ROS_WARN("OccupancyGrid::RayTrace(), robot not in range");
    return;
  }

  misc_utils_ns::UniquifyIntVector(updated_grid_indices_);

  for (const auto& ind : updated_grid_indices_)
  {
    if (grid_array_->InRange(ind))
    {
      Eigen::Vector3i cur_sub = grid_array_->Ind2Sub(ind);
      if (!grid_array_->InRange(cur_sub))
      {
        ROS_WARN_STREAM("OccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub out of range");
        continue;
      }
      if (grid_array_->GetCellValue(cur_sub) != OCCUPIED)
      {
        ROS_WARN_STREAM("OccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub not occupied");
        continue;
      }
      ray_trace_count++;
      std::vector<Eigen::Vector3i> ray_cast_cells;
      RayTraceHelper(origin_sub, cur_sub, ray_cast_cells);
      for (int i = 0; i < ray_cast_cells.size(); i++)
      {
        Eigen::Vector3i ray_sub = ray_cast_cells[i];
        if (grid_array_->GetCellValue(ray_sub) == OCCUPIED)
        {
          break;
        }
        else
        {
          if (grid_array_->GetCellValue(ray_sub) != OCCUPIED)
          {
            grid_array_->SetCellValue(ray_sub, FREE);
          }
        }
      }
    }
  }
}

void OccupancyGrid::RayTrace(const Eigen::Vector3d& origin)
{
  RayTrace(origin, range_);
}

void OccupancyGrid::GetVisualizationCloudInRange(const Eigen::Vector3d& origin, const Eigen::Vector3d& range,
                                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& occupied_cloud,
                                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& free_cloud,
                                                 pcl::PointCloud<pcl::PointXYZI>::Ptr& unknown_cloud)
{
  occupied_cloud->points.clear();
  free_cloud->points.clear();
  unknown_cloud->points.clear();

  Eigen::Vector3i sub_max = grid_array_->Pos2Sub(origin + range);
  Eigen::Vector3i sub_min = grid_array_->Pos2Sub(origin - range);
  Eigen::Vector3i origin_sub = grid_array_->Pos2Sub(origin);
  if (!grid_array_->InRange(origin_sub))
  {
    ROS_WARN("OccupancyGrid::GetVisualizationCloudInRange(), robot not in range");
    return;
  }
  int occupied_count = 0;
  int free_count = 0;
  int unknown_count = 0;
  for (int sub_x = sub_min.x(); sub_x <= sub_max.x(); sub_x++)
  {
    for (int sub_y = sub_min.y(); sub_y <= sub_max.y(); sub_y++)
    {
      for (int sub_z = sub_min.z(); sub_z <= sub_max.z(); sub_z++)
      {
        Eigen::Vector3i cur_sub(sub_x, sub_y, sub_z);
        if (grid_array_->InRange(cur_sub))
        {
          Eigen::Vector3d point_position = grid_array_->Sub2Pos(cur_sub);
          pcl::PointXYZI point;
          point.x = point_position.x();
          point.y = point_position.y();
          point.z = point_position.z();
          point.intensity = 0.0;
          if (grid_array_->GetCellValue(cur_sub) == OCCUPIED)
          {
            occupied_cloud->points.push_back(point);
            occupied_count++;
          }
          else if (grid_array_->GetCellValue(cur_sub) == FREE)
          {
            free_cloud->points.push_back(point);
            free_count++;
          }
          else
          {
            unknown_cloud->points.push_back(point);
            unknown_count++;
          }
        }
      }
    }
  }
}

void OccupancyGrid::GetFrontierInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr& frontier_cloud,
                                       const Eigen::Vector3d& origin, const Eigen::Vector3d& range)
{
  frontier_cloud->points.clear();
  Eigen::Vector3i sub_max = grid_array_->Pos2Sub(origin + range);
  Eigen::Vector3i sub_min = grid_array_->Pos2Sub(origin - range);
  Eigen::Vector3i origin_sub = grid_array_->Pos2Sub(origin);
  Eigen::Vector3i eliminate_frontier_origin_sub = grid_array_->Pos2Sub(eliminate_frontier_origin_);

  if (!grid_array_->InRange(origin_sub))
  {
    ROS_WARN("OccupancyGrid::GetFrontierInRange(), robot not in range");
    return;
  }
  int ray_trace_count = 0;

  for (int sub_x = sub_min.x(); sub_x <= sub_max.x(); sub_x++)
  {
    for (int sub_y = sub_min.y(); sub_y <= sub_max.y(); sub_y++)
    {
      for (int sub_z = sub_min.z(); sub_z <= sub_max.z(); sub_z++)
      {
        Eigen::Vector3i cur_sub(sub_x, sub_y, sub_z);
        if (!grid_array_->InRange(cur_sub))
        {
          continue;
        }
        if (grid_array_->GetCellValue(cur_sub) == UNKNOWN)
        {
          bool z_free = false;
          bool xy_free = false;
          // If the unknown cell has neighboring free cells in xy but not z direction
          cur_sub(2)--;
          if (grid_array_->InRange(cur_sub) && grid_array_->GetCellValue(cur_sub) == FREE)
          {
            z_free = true;
            continue;
          }
          cur_sub(2) += 2;
          if (grid_array_->InRange(cur_sub) && grid_array_->GetCellValue(cur_sub) == FREE)
          {
            z_free = true;
            continue;
          }
          cur_sub(2)--;

          for (int i = 0; i < 2; i++)
          {
            cur_sub(i)--;
            if (grid_array_->InRange(cur_sub) && grid_array_->GetCellValue(cur_sub) == FREE)
            {
              xy_free = true;
              cur_sub(i)++;
              break;
            }
            cur_sub(i) += 2;
            if (grid_array_->InRange(cur_sub) && grid_array_->GetCellValue(cur_sub) == FREE)
            {
              xy_free = true;
              cur_sub(i)--;
              break;
            }
            cur_sub(i)--;
          }
          if (xy_free && !z_free)
          {
            if (eliminate_frontier_ && CanRayTrace(eliminate_frontier_origin_sub, cur_sub))
            {
              grid_array_->SetCellValue(cur_sub, NOT_FRONTIER);
            }
            else
            {
              Eigen::Vector3d position = grid_array_->Sub2Pos(cur_sub);
              pcl::PointXYZI point;
              point.x = position.x();
              point.y = position.y();
              point.z = position.z();
              point.intensity = 0;
              frontier_cloud->points.push_back(point);
            }
          }
        }
      }
    }
  }
}

void OccupancyGrid::GetFrontierInRange(pcl::PointCloud<pcl::PointXYZI>::Ptr& frontier_cloud,
                                       const Eigen::Vector3d& origin)
{
  GetFrontierInRange(frontier_cloud, origin, range_);
}

}  // namespace occupancy_grid_ns