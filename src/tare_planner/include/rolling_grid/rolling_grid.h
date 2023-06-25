/**
 * @file rolling_grid.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a rolling 3D grid
 * @version 0.1
 * @date 2020-06-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <algorithm>
#include <cmath>

#include <Eigen/Core>

#include <grid/grid.h>
#include <utils/misc_utils.h>

namespace rolling_grid_ns
{
class RollingGrid
{
public:
  explicit RollingGrid(const Eigen::Vector3i& size);
  ~RollingGrid() = default;
  bool InRange(Eigen::Vector3i sub) const
  {
    return grid0_->InRange(sub);
  }
  bool InRange(int ind) const
  {
    return grid0_->InRange(ind);
  }
  Eigen::Vector3i Ind2Sub(int ind) const
  {
    return grid0_->Ind2Sub(ind);
  }
  int Sub2Ind(Eigen::Vector3i sub) const
  {
    return grid0_->Sub2Ind(sub);
  }
  int GetArrayInd(Eigen::Vector3i sub) const
  {
    if (!InRange(sub))
    {
      std::cout << "out of range sub: " << sub.transpose() << std::endl;
    }
    MY_ASSERT(InRange(sub));
    if (which_grid_)
    {
      return grid1_->GetCellValue(sub);
    }
    else
    {
      return grid0_->GetCellValue(sub);
    }
  }
  int GetArrayInd(int ind) const
  {
    if (!InRange(ind))
    {
      std::cout << "out of range ind: " << ind << std::endl;
    }
    MY_ASSERT(InRange(ind));
    Eigen::Vector3i sub = grid0_->Ind2Sub(ind);
    return GetArrayInd(sub);
  }
  int GetInd(int array_ind) const
  {
    MY_ASSERT(InRange(array_ind));
    return array_ind_to_ind_[array_ind];
  }

  void Roll(const Eigen::Vector3i& roll_dir);
  void GetUpdatedIndices(std::vector<int>& updated_indices) const;
  void GetRolledOutIndices(const Eigen::Vector3i& roll_dir, std::vector<int>& rolled_out_indices);
  void GetUpdatedArrayIndices(std::vector<int>& updated_array_indices) const;

private:
  Eigen::Vector3i size_;
  std::shared_ptr<grid_ns::Grid<int>> grid0_;
  std::shared_ptr<grid_ns::Grid<int>> grid1_;
  std::vector<int> updated_indices_;
  std::vector<int> array_ind_to_ind_;
  bool which_grid_;

  inline int GetFromIdx(int cur_idx, int roll_step, int max_idx) const
  {
    return cur_idx <= roll_step - 1 ? max_idx - roll_step + cur_idx : cur_idx - roll_step;
  }
  void RollHelper(const std::shared_ptr<grid_ns::Grid<int>>& grid_in,
                  const std::shared_ptr<grid_ns::Grid<int>>& grid_out, Eigen::Vector3i roll_dir);

  void GetRolledInIndices(const Eigen::Vector3i& roll_dir);
  void GetRolledOutIndices(const Eigen::Vector3i& roll_dir);
  void GetIndices(std::vector<int>& indices, Eigen::Vector3i start_idx, Eigen::Vector3i end_idx) const;
};
}  // namespace rolling_grid_ns