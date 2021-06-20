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
RollingOccupancyGrid::RollingOccupancyGrid(ros::NodeHandle& nh) : initialized_(false), dimension_(3)
{
  range_.x() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/range_x", 100);
  range_.y() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/range_y", 100);
  range_.z() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/range_z", 10);

  resolution_.x() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/resolution_x", 0.2);
  resolution_.y() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/resolution_y", 0.2);
  resolution_.z() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/resolution_z", 0.2);

  rollover_range_.x() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/rollover_range_x", 20);
  rollover_range_.y() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/rollover_range_y", 20);
  rollover_range_.z() = misc_utils_ns::getParam<double>(nh, "rolling_occupancy_grid/rollover_range_z", 2);

  for (int i = 0; i < dimension_; i++)
  {
    grid_size_(i) = static_cast<int>(range_(i) / resolution_(i));
    rollover_step_size_(i) = static_cast<int>(rollover_range_(i) / resolution_(i));
    origin_(i) = -range_(i) / 2;
  }
  std::cout << "grid size: " << grid_size_.transpose() << std::endl;
  std::cout << "rollover step size: " << rollover_step_size_.transpose() << std::endl;
  std::cout << "origin: " << origin_.transpose() << std::endl;

  rolling_grid_ = std::make_unique<rollable_grid_ns::RollableGrid>(grid_size_);
  occupancy_array_ = std::make_unique<grid_ns::Grid<CellState>>(grid_size_, UNKNOWN, origin_, resolution_);

  robot_position_ = Eigen::Vector3d(0, 0, 0);
}

void RollingOccupancyGrid::InitializeOrigin()
{
  for (int i = 0; i < dimension_; i++)
  {
    origin_(i) = robot_position_(i) - range_(i) / 2;
  }
  occupancy_array_->SetOrigin(origin_);
}

bool RollingOccupancyGrid::UpdateRobotPosition(const Eigen::Vector3d& robot_position)
{
  robot_position_ = robot_position;
  if (!initialized_)
  {
    initialized_ = true;
    InitializeOrigin();
  }
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
  misc_utils_ns::Timer rolling_grid_timer("rolling grid");
  rolling_grid_timer.Start();
  rolling_grid_->Roll(rollover_step);
  rolling_grid_timer.Stop(true);

  for (int i = 0; i < dimension_; i++)
  {
    origin_(i) -= rollover_step(i) * resolution_(i);
  }

  occupancy_array_->SetOrigin(origin_);

  // Update rolled over memory
  misc_utils_ns::Timer getting_update_indices_timer("updating indices");
  getting_update_indices_timer.Start();
  std::vector<int> updated_grid_array_indices;
  rolling_grid_->GetUpdatedArrayIndices(updated_grid_array_indices);
  getting_update_indices_timer.Stop(true);

  std::cout << "updated array list size: " << updated_grid_array_indices.size() << std::endl;

  // Todo: Store the occupancy status to pointcloud
  misc_utils_ns::Timer reset_value_timer("resetting grid value");
  reset_value_timer.Start();
  for (const auto& ind : updated_grid_array_indices)
  {
    occupancy_array_->SetCellValue(ind, CellState::UNKNOWN);
  }
  reset_value_timer.Stop(true);

  return true;
}

void RollingOccupancyGrid::RayTrace(const Eigen::Vector3d& origin, const Eigen::Vector3d& range)
{
  Eigen::Vector3i sub_max = occupancy_array_->GetSize() - Eigen::Vector3i::Ones();
  Eigen::Vector3i sub_min = Eigen::Vector3i(0, 0, 0);
  Eigen::Vector3i origin_sub = occupancy_array_->Pos2Sub(origin);
  int ray_trace_count = 0;
  if (!occupancy_array_->InRange(origin_sub))
  {
    ROS_WARN("RollingOccupancyGrid::RayTrace(), robot not in range");
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
        ROS_WARN_STREAM("RollingOccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub out of range");
        continue;
      }
      int array_ind = rolling_grid_->GetArrayInd(ind);
      if (occupancy_array_->GetCellValue(array_ind) != OCCUPIED)
      {
        ROS_WARN_STREAM("RollingOccupancyGrid::RayTrace() " << cur_sub.transpose() << " sub not occupied");
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

  while (true)
  {
    MY_ASSERT(occupancy_array_->InRange(cur_sub));
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
      else
      {
        point.intensity = 1.0;
      }
      vis_cloud->points.push_back(point);
    }
  }
}

}  // namespace rolling_occupancy_grid_ns