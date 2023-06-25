/**
 * @file rolling_occupancy_grid.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a rolling occupancy grid
 * @version 0.1
 * @date 2021-06-16
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "rolling_occupancy_grid/rolling_occupancy_grid.h"

namespace rolling_occupancy_grid_ns
{
RollingOccupancyGrid::RollingOccupancyGrid(rclcpp::Node::SharedPtr nh) : initialized_(false), dimension_(3)
{
  double pointcloud_cell_size = nh->get_parameter("kPointCloudCellSize").as_double();
  double pointcloud_cell_height = nh->get_parameter("kPointCloudCellHeight").as_double();
  int pointcloud_cell_neighbor_number = nh->get_parameter("kPointCloudManagerNeighborCellNum").as_int();

  range_.x() = pointcloud_cell_size * pointcloud_cell_neighbor_number;
  range_.y() = pointcloud_cell_size * pointcloud_cell_neighbor_number;
  range_.z() = pointcloud_cell_height * pointcloud_cell_neighbor_number;

  resolution_.x() = nh->get_parameter("rolling_occupancy_grid/resolution_x").as_double();
  resolution_.y() = nh->get_parameter("rolling_occupancy_grid/resolution_y").as_double();
  resolution_.z() = nh->get_parameter("rolling_occupancy_grid/resolution_z").as_double();

  rollover_range_.x() = pointcloud_cell_size;
  rollover_range_.y() = pointcloud_cell_size;
  rollover_range_.z() = pointcloud_cell_height;

  for (int i = 0; i < dimension_; i++)
  {
    grid_size_(i) = static_cast<int>(range_(i) / resolution_(i));
    rollover_step_size_(i) = static_cast<int>(rollover_range_(i) / resolution_(i));
    origin_(i) = -range_(i) / 2;
  }

  rolling_grid_ = std::make_shared<rolling_grid_ns::RollingGrid>(grid_size_);

  occupancy_array_ = std::make_shared<grid_ns::Grid<CellState>>(grid_size_, UNKNOWN, origin_, resolution_);

  robot_position_ = Eigen::Vector3d(0, 0, 0);

  occupancy_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

void RollingOccupancyGrid::InitializeOrigin(const Eigen::Vector3d& origin)
{
  if (!initialized_)
  {
    initialized_ = true;
    origin_ = origin;
    occupancy_array_->SetOrigin(origin_);
  }
}

bool RollingOccupancyGrid::UpdateRobotPosition(const Eigen::Vector3d& robot_position)
{
  if (!initialized_)
  {
    return false;
  }
  robot_position_ = robot_position;
  Eigen::Vector3i robot_grid_sub;
  Eigen::Vector3d diff = robot_position_ - origin_;
  Eigen::Vector3i sub = Eigen::Vector3i::Zero();
  for (int i = 0; i < dimension_; i++)
  {
    robot_grid_sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / (rollover_step_size_(i) * resolution_(i))) : -1;
  }

  Eigen::Vector3i sub_diff = Eigen::Vector3i::Zero();
  for (int i = 0; i < dimension_; i++)
  {
    sub_diff(i) = (grid_size_(i) / rollover_step_size_(i)) / 2 - robot_grid_sub(i);
  }

  if (sub_diff.x() == 0 && sub_diff.y() == 0 && sub_diff.z() == 0)
  {
    return false;
  }

  Eigen::Vector3i rollover_step(0, 0, 0);
  for (int i = 0; i < dimension_; i++)
  {
    rollover_step(i) =
        std::abs(sub_diff(i)) > 0 ? rollover_step_size_(i) * ((sub_diff(i) > 0) ? 1 : -1) * std::abs(sub_diff(i)) : 0;
  }

  std::vector<int> rolled_out_grid_indices;
  rolling_grid_->GetRolledOutIndices(rollover_step, rolled_out_grid_indices);

  // Get rolled out occupancy cloud
  occupancy_cloud_->clear();
  for (const auto& ind : rolled_out_grid_indices)
  {
    int array_ind = rolling_grid_->GetArrayInd(ind);
    if (occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(ind);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = 0;
      occupancy_cloud_->points.push_back(point);
    }
    else if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED)
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(ind);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = 1;
      occupancy_cloud_->points.push_back(point);
    }
  }

  rolling_grid_->Roll(rollover_step);

  // Update origin
  for (int i = 0; i < dimension_; i++)
  {
    origin_(i) -= rollover_step(i) * resolution_(i);
  }
  occupancy_array_->SetOrigin(origin_);

  std::vector<int> updated_grid_indices;
  rolling_grid_->GetUpdatedArrayIndices(updated_grid_indices);

  for (const auto& ind : updated_grid_indices)
  {
    occupancy_array_->SetCellValue(ind, CellState::UNKNOWN);
  }

  return true;
}

void RollingOccupancyGrid::UpdateOccupancyStatus(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  if (!initialized_)
  {
    return;
  }
  for (const auto& point : cloud->points)
  {
    Eigen::Vector3i sub = occupancy_array_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
    if (!occupancy_array_->InRange(sub))
    {
      continue;
    }
    int array_ind = rolling_grid_->GetArrayInd(sub);
    if (point.intensity < 0.1)
    {
      occupancy_array_->SetCellValue(array_ind, CellState::FREE);
    }
    else if (point.intensity > 0.9)
    {
      occupancy_array_->SetCellValue(array_ind, CellState::OCCUPIED);
    }
  }
}

void RollingOccupancyGrid::RayTrace(const Eigen::Vector3d& origin, const Eigen::Vector3d& range)
{
  // Eigen::Vector3i sub_max = occupancy_array_->GetSize() - Eigen::Vector3i::Ones();
  // Eigen::Vector3i sub_min = Eigen::Vector3i(0, 0, 0);
  Eigen::Vector3i origin_sub = occupancy_array_->Pos2Sub(origin);
  int ray_trace_count = 0;
  if (!occupancy_array_->InRange(origin_sub))
  {
    RCLCPP_WARN(rclcpp::get_logger("standalone_logger"), "RollingOccupancyGrid::RayTrace(), robot not in range");
    return;
  }

  misc_utils_ns::UniquifyIntVector(updated_grid_indices_);

  for (const auto& ind : updated_grid_indices_)
  {
    if (occupancy_array_->InRange(ind))
    {
      Eigen::Vector3i cur_sub = occupancy_array_->Ind2Sub(ind);
      if (!occupancy_array_->InRange(cur_sub))
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("standalone_logger"),
                           "RollingOccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub out of range");
        continue;
      }
      int array_ind = rolling_grid_->GetArrayInd(ind);
      if (occupancy_array_->GetCellValue(array_ind) != OCCUPIED)
      {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("standalone_logger"),
                           "RollingOccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub not occupied");
        continue;
      }
      ray_trace_count++;
      std::vector<Eigen::Vector3i> ray_cast_cells;
      RayTraceHelper(origin_sub, cur_sub, ray_cast_cells);
      for (int i = 0; i < ray_cast_cells.size(); i++)
      {
        Eigen::Vector3i ray_sub = ray_cast_cells[i];
        int array_ind = rolling_grid_->GetArrayInd(ray_sub);
        if (occupancy_array_->GetCellValue(array_ind) == OCCUPIED)
        {
          break;
        }
        else
        {
          if (occupancy_array_->GetCellValue(array_ind) != OCCUPIED)
          {
            occupancy_array_->SetCellValue(array_ind, FREE);
          }
        }
      }
    }
  }
}

void RollingOccupancyGrid::RayTrace(const Eigen::Vector3d& origin)
{
  if (!initialized_)
  {
    return;
  }
  RayTrace(origin, range_);
}

void RollingOccupancyGrid::RayTraceHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                          std::vector<Eigen::Vector3i>& cells)
{
  cells.clear();
  MY_ASSERT(occupancy_array_->InRange(start_sub));
  MY_ASSERT(occupancy_array_->InRange(end_sub));

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

  while (occupancy_array_->InRange(cur_sub))
  {
    cells.push_back(cur_sub);
    dist = (cur_sub - start_sub).squaredNorm();
    int array_ind = rolling_grid_->GetArrayInd(cur_sub);
    if (cur_sub == end_sub || dist > max_dist || occupancy_array_->GetCellValue(array_ind) == OCCUPIED)
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

void RollingOccupancyGrid::GetFrontier(pcl::PointCloud<pcl::PointXYZI>::Ptr& frontier_cloud,
                                       const Eigen::Vector3d& origin, const Eigen::Vector3d& range)
{
  if (!initialized_)
  {
    return;
  }
  frontier_cloud->points.clear();
  Eigen::Vector3i sub_max = occupancy_array_->Pos2Sub(origin + range);
  Eigen::Vector3i sub_min = occupancy_array_->Pos2Sub(origin - range);
  Eigen::Vector3i origin_sub = occupancy_array_->Pos2Sub(origin);

  if (!occupancy_array_->InRange(origin_sub))
  {
    RCLCPP_WARN(rclcpp::get_logger("standalone_logger"),
                "RollingOccupancyGrid::GetFrontierInRange(), robot not in range");
    return;
  }
  int ray_trace_count = 0;

  int cell_num = occupancy_array_->GetCellNumber();
  for (int ind = 0; ind < cell_num; ind++)
  {
    Eigen::Vector3i cur_sub = occupancy_array_->Ind2Sub(ind);
    if (!occupancy_array_->InRange(cur_sub))
    {
      continue;
    }
    if (!InRange(cur_sub, sub_min, sub_max))
    {
      continue;
    }
    int array_ind = rolling_grid_->GetArrayInd(cur_sub);
    if (occupancy_array_->GetCellValue(array_ind) == UNKNOWN)
    {
      bool z_free = false;
      bool xy_free = false;
      // If the unknown cell has neighboring free cells in xy but not z direction
      cur_sub(2)--;
      if (occupancy_array_->InRange(cur_sub))
      {
        array_ind = rolling_grid_->GetArrayInd(cur_sub);
        if (occupancy_array_->GetCellValue(array_ind) == FREE)
        {
          z_free = true;
          continue;
        }
      }
      cur_sub(2) += 2;
      if (occupancy_array_->InRange(cur_sub))
      {
        array_ind = rolling_grid_->GetArrayInd(cur_sub);
        if (occupancy_array_->GetCellValue(array_ind) == FREE)
        {
          z_free = true;
          continue;
        }
      }
      cur_sub(2)--;

      for (int i = 0; i < 2; i++)
      {
        cur_sub(i)--;
        if (occupancy_array_->InRange(cur_sub))
        {
          array_ind = rolling_grid_->GetArrayInd(cur_sub);
          if (occupancy_array_->GetCellValue(array_ind) == FREE)
          {
            xy_free = true;
            cur_sub(i)++;
            break;
          }
        }
        cur_sub(i) += 2;
        if (occupancy_array_->InRange(cur_sub))
        {
          array_ind = rolling_grid_->GetArrayInd(cur_sub);
          if (occupancy_array_->GetCellValue(array_ind) == FREE)
          {
            xy_free = true;
            cur_sub(i)--;
            break;
          }
        }
        cur_sub(i)--;
      }
      if (xy_free && !z_free)
      {
        Eigen::Vector3d position = occupancy_array_->Sub2Pos(cur_sub);
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

void RollingOccupancyGrid::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->clear();
  int cell_number = occupancy_array_->GetCellNumber();
  for (int i = 0; i < cell_number; i++)
  {
    int array_ind = rolling_grid_->GetArrayInd(i);
    if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED ||
        occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
    {
      Eigen::Vector3d position = occupancy_array_->Ind2Pos(i);
      pcl::PointXYZI point;
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      if (occupancy_array_->GetCellValue(array_ind) == CellState::OCCUPIED)
      {
        point.intensity = 0.0;
      }
      else if (occupancy_array_->GetCellValue(array_ind) == CellState::FREE)
      {
        point.intensity = 1.0;
      }
      else
      {
        point.intensity = 2.0;
      }
      vis_cloud->points.push_back(point);
    }
  }
}

bool RollingOccupancyGrid::InRange(const Eigen::Vector3i& sub, const Eigen::Vector3i& sub_min,
                                   const Eigen::Vector3i& sub_max)
{
  bool in_range = true;
  for (int i = 0; i < 3; i++)
  {
    in_range &= (sub(i) >= sub_min(i) && sub(i) <= sub_max(i));
  }
  return in_range;
}

}  // namespace rolling_occupancy_grid_ns