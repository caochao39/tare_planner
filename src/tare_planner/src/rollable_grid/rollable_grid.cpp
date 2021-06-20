/**
 * @file rollable_grid.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a rollable 3D grid
 * @version 0.1
 * @date 2020-06-10
 *
 * @copyright Copyright (c) 2021
 *
 */

#include <rollable_grid/rollable_grid.h>

namespace rollable_grid_ns
{
RollableGrid::RollableGrid(const Eigen::Vector3i size) : size_(size), which_grid_(true)
{
  array_ind_to_ind_.resize(size_.x() * size_.y() * size_.z());
  grid0_ = std::make_unique<grid_ns::Grid<int>>(size_, 0);
  grid1_ = std::make_unique<grid_ns::Grid<int>>(size_, 0);
  // Initialize array indices
  for (int x = 0; x < size_.x(); x++)
  {
    for (int y = 0; y < size_.y(); y++)
    {
      for (int z = 0; z < size_.z(); z++)
      {
        int ind = grid0_->Sub2Ind(x, y, z);
        grid0_->GetCell(x, y, z) = ind;
        grid1_->GetCell(x, y, z) = ind;
        array_ind_to_ind_[ind] = ind;
      }
    }
  }
}

void RollableGrid::Roll(Eigen::Vector3i roll_dir)
{
  if (roll_dir.x() == 0 && roll_dir.y() == 0 && roll_dir.z() == 0)
  {
    return;
  }
  misc_utils_ns::Timer roll_helper_timer("-roll helper");
  roll_helper_timer.Start();
  if (which_grid_)
  {
    RollHelper(grid1_, grid0_, roll_dir);
  }
  else
  {
    RollHelper(grid0_, grid1_, roll_dir);
  }
  roll_helper_timer.Stop(true);

  which_grid_ = !which_grid_;

  misc_utils_ns::Timer update_mapping_timer("-update mapping");
  update_mapping_timer.Start();
  // Update array_ind to ind mapping
  for (int x = 0; x < size_.x(); x++)
  {
    for (int y = 0; y < size_.y(); y++)
    {
      for (int z = 0; z < size_.z(); z++)
      {
        int ind = grid0_->Sub2Ind(x, y, z);
        int array_ind;
        if (which_grid_)
        {
          array_ind = grid1_->GetCellValue(x, y, z);
        }
        else
        {
          array_ind = grid0_->GetCellValue(x, y, z);
        }
        array_ind_to_ind_[array_ind] = ind;
      }
    }
  }
  update_mapping_timer.Stop(true);
}

void RollableGrid::RollHelper(const std::unique_ptr<grid_ns::Grid<int>>& grid_in,
                              const std::unique_ptr<grid_ns::Grid<int>>& grid_out, Eigen::Vector3i roll_dir)
{
  Eigen::Vector3i grid_in_size = grid_in->GetSize();
  Eigen::Vector3i grid_out_size = grid_out->GetSize();
  MY_ASSERT(grid_in_size.x() == grid_out_size.x() && grid_in_size.x() == size_.x());
  MY_ASSERT(grid_in_size.y() == grid_out_size.y() && grid_in_size.y() == size_.y());
  MY_ASSERT(grid_in_size.z() == grid_out_size.z() && grid_in_size.z() == size_.z());
  roll_dir.x() %= size_.x();
  roll_dir.y() %= size_.y();
  roll_dir.z() %= size_.z();
  Eigen::Vector3i dir;
  dir.x() = roll_dir.x() >= 0 ? roll_dir.x() : size_.x() + roll_dir.x();
  dir.y() = roll_dir.y() >= 0 ? roll_dir.y() : size_.y() + roll_dir.y();
  dir.z() = roll_dir.z() >= 0 ? roll_dir.z() : size_.z() + roll_dir.z();

  misc_utils_ns::Timer update_index_timer("--update grid indices");
  update_index_timer.Start();
  for (int x = 0; x < size_.x(); x++)
  {
    for (int y = 0; y < size_.y(); y++)
    {
      for (int z = 0; z < size_.z(); z++)
      {
        int from_x = GetFromIdx(x, dir.x(), size_.x());
        int from_y = GetFromIdx(y, dir.y(), size_.y());
        int from_z = GetFromIdx(z, dir.z(), size_.z());
        grid_out->GetCell(x, y, z) = grid_in->GetCellValue(from_x, from_y, from_z);
      }
    }
  }
  update_index_timer.Stop(true);

  Eigen::Vector3i start_idx, end_idx;
  start_idx.x() = roll_dir.x() >= 0 ? 0 : size_.x() + roll_dir.x();
  start_idx.y() = roll_dir.y() >= 0 ? 0 : size_.y() + roll_dir.y();
  start_idx.z() = roll_dir.z() >= 0 ? 0 : size_.z() + roll_dir.z();

  end_idx.x() = roll_dir.x() >= 0 ? roll_dir.x() - 1 : size_.x() - 1;
  end_idx.y() = roll_dir.y() >= 0 ? roll_dir.y() - 1 : size_.y() - 1;
  end_idx.z() = roll_dir.z() >= 0 ? roll_dir.z() - 1 : size_.z() - 1;

  misc_utils_ns::Timer get_indices_timer("--get indices");
  get_indices_timer.Start();
  updated_indices_.clear();
  if (dir.x() > 0)
  {
    GetIndices(updated_indices_, Eigen::Vector3i(start_idx.x(), 0, 0),
               Eigen::Vector3i(end_idx.x(), size_.y() - 1, size_.z() - 1));
  }
  if (dir.y() > 0)
  {
    int x_start = 0;
    int x_end = size_.x() - 1;
    if (start_idx.x() == 0)
    {
      x_start = end_idx.x() + 1;
      x_end = size_.x() - 1;
    }
    else
    {
      x_start = 0;
      x_end = start_idx.x() - 1;
    }
    GetIndices(updated_indices_, Eigen::Vector3i(x_start, start_idx.y(), 0),
               Eigen::Vector3i(x_end, end_idx.y(), size_.z() - 1));
  }
  if (dir.z() > 0)
  {
    int x_start = 0;
    int x_end = size_.x() - 1;
    int y_start = 0;
    int y_end = size_.y() - 1;
    if (start_idx.x() == 0)
    {
      x_start = end_idx.x() + 1;
      x_end = size_.x() - 1;
    }
    else
    {
      x_start = 0;
      x_end = start_idx.x() - 1;
    }
    if (start_idx.y() == 0)
    {
      y_start = end_idx.y() + 1;
      y_end = size_.y() - 1;
    }
    else
    {
      y_start = 0;
      y_end = start_idx.y() - 1;
    }
    GetIndices(updated_indices_, Eigen::Vector3i(x_start, y_start, start_idx.z()),
               Eigen::Vector3i(x_end, y_end, end_idx.z()));
  }

  get_indices_timer.Stop(true);
}

void RollableGrid::GetIndices(std::vector<int>& indices, Eigen::Vector3i start_idx, Eigen::Vector3i end_idx) const
{
  start_idx.x() %= size_.x();
  start_idx.y() %= size_.y();
  start_idx.z() %= size_.z();

  end_idx.x() %= size_.x();
  end_idx.y() %= size_.y();
  end_idx.z() %= size_.z();

  start_idx.x() = std::min(start_idx.x(), end_idx.x());
  start_idx.y() = std::min(start_idx.y(), end_idx.y());
  start_idx.z() = std::min(start_idx.z(), end_idx.z());

  end_idx.x() = std::max(start_idx.x(), end_idx.x());
  end_idx.y() = std::max(start_idx.y(), end_idx.y());
  end_idx.z() = std::max(start_idx.z(), end_idx.z());

  for (int x = start_idx.x(); x <= end_idx.x(); x++)
  {
    for (int y = start_idx.y(); y <= end_idx.y(); y++)
    {
      for (int z = start_idx.z(); z <= end_idx.z(); z++)
      {
        indices.push_back(grid0_->Sub2Ind(x, y, z));
      }
    }
  }
}

void RollableGrid::GetUpdatedIndices(std::vector<int>& updated_indices) const
{
  updated_indices.clear();
  updated_indices.resize(updated_indices_.size());
  std::copy(updated_indices_.begin(), updated_indices_.end(), updated_indices.begin());
}

void RollableGrid::GetUpdatedArrayIndices(std::vector<int>& updated_array_indices) const
{
  updated_array_indices.clear();
  for (const auto& ind : updated_indices_)
  {
    Eigen::Vector3i sub = grid0_->Ind2Sub(ind);
    updated_array_indices.push_back(GetArrayInd(sub));
  }
}
}  // namespace rollable_grid_ns