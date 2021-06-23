/**
 * @file grid.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a 3D grid
 * @version 0.1
 * @date 2021-01-26
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <Eigen/Core>
#include <utils/misc_utils.h>

namespace grid_ns
{
template <typename _T>
class Grid
{
public:
  explicit Grid(const Eigen::Vector3i& size, _T init_value, const Eigen::Vector3d& origin = Eigen::Vector3d(0, 0, 0),
                const Eigen::Vector3d& resolution = Eigen::Vector3d(1, 1, 1), int dimension = 3)
  {
    // MY_ASSERT(size.x() > 0);
    // MY_ASSERT(size.y() > 0);
    // MY_ASSERT(size.z() > 0);

    origin_ = origin;
    size_ = size;
    resolution_ = resolution;
    dimension_ = dimension;

    for (int i = 0; i < dimension_; i++)
    {
      resolution_inv_(i) = 1.0 / resolution_(i);
    }
    cell_number_ = size_.x() * size_.y() * size_.z();
    for (int i = 0; i < cell_number_; i++)
    {
      cells_.push_back(init_value);
      subs_.push_back(ind2sub_(i));
    }
  }

  virtual ~Grid() = default;

  int GetCellNumber() const
  {
    return cell_number_;
  }

  Eigen::Vector3i GetSize() const
  {
    return size_;
  }

  Eigen::Vector3d GetOrigin() const
  {
    return origin_;
  }

  void SetOrigin(const Eigen::Vector3d& origin)
  {
    origin_ = origin;
  }

  void SetResolution(const Eigen::Vector3d& resolution)
  {
    resolution_ = resolution;
    for (int i = 0; i < dimension_; i++)
    {
      resolution_inv_(i) = 1.0 / resolution(i);
    }
  }

  Eigen::Vector3d GetResolution() const
  {
    return resolution_;
  }

  Eigen::Vector3d GetResolutionInv() const
  {
    return resolution_inv_;
  }

  bool InRange(int x, int y, int z) const
  {
    return InRange(Eigen::Vector3i(x, y, z));
  }

  bool InRange(const Eigen::Vector3i& sub) const
  {
    bool in_range = true;
    for (int i = 0; i < dimension_; i++)
    {
      in_range &= sub(i) >= 0 && sub(i) < size_(i);
    }
    return in_range;
  }

  bool InRange(int ind) const
  {
    return ind >= 0 && ind < cell_number_;
  }

  Eigen::Vector3i Ind2Sub(int ind) const
  {
    // MY_ASSERT(InRange(ind));
    return subs_[ind];
  }

  int Sub2Ind(int x, int y, int z) const
  {
    return x + (y * size_.x()) + (z * size_.x() * size_.y());
  }

  int Sub2Ind(const Eigen::Vector3i& sub) const
  {
    // MY_ASSERT(InRange(sub));
    return Sub2Ind(sub.x(), sub.y(), sub.z());
  }

  Eigen::Vector3d Sub2Pos(int x, int y, int z) const
  {
    return Sub2Pos(Eigen::Vector3i(x, y, z));
  }

  Eigen::Vector3d Sub2Pos(const Eigen::Vector3i& sub) const
  {
    Eigen::Vector3d pos(0, 0, 0);
    for (int i = 0; i < dimension_; i++)
    {
      pos(i) = origin_(i) + sub(i) * resolution_(i) + resolution_(i) / 2;
    }
    return pos;
  }

  Eigen::Vector3d Ind2Pos(int ind) const
  {
    // MY_ASSERT(InRange(ind));
    return Sub2Pos(Ind2Sub(ind));
  }

  Eigen::Vector3i Pos2Sub(double x, double y, double z) const
  {
    return Pos2Sub(Eigen::Vector3d(x, y, z));
  }

  Eigen::Vector3i Pos2Sub(const Eigen::Vector3d& pos) const
  {
    Eigen::Vector3i sub(0, 0, 0);
    for (int i = 0; i < dimension_; i++)
    {
      sub(i) = pos(i) - origin_(i) > 0 ? static_cast<int>((pos(i) - origin_(i)) * resolution_inv_(i)) : -1;
    }
    return sub;
  }

  int Pos2Ind(const Eigen::Vector3d& pos) const
  {
    return Sub2Ind(Pos2Sub(pos));
  }

  _T& GetCell(int x, int y, int z)
  {
    return GetCell(Eigen::Vector3i(x, y, z));
  }

  _T& GetCell(const Eigen::Vector3i& sub)
  {
    // MY_ASSERT(InRange(sub));
    int index = Sub2Ind(sub);
    return cells_[index];
  }

  _T& GetCell(int index)
  {
    // MY_ASSERT(InRange(index));
    return cells_[index];
  }

  _T GetCellValue(int x, int y, int z) const
  {
    int index = Sub2Ind(x, y, z);
    return cells_[index];
  }

  _T GetCellValue(const Eigen::Vector3i& sub) const
  {
    // MY_ASSERT(InRange(sub));
    return GetCellValue(sub.x(), sub.y(), sub.z());
  }

  _T GetCellValue(int index) const
  {
    // MY_ASSERT(InRange(index));
    return cells_[index];
  }

  void SetCellValue(int x, int y, int z, _T value)
  {
    int index = Sub2Ind(x, y, z);
    cells_[index] = value;
  }

  void SetCellValue(const Eigen::Vector3i& sub, _T value)
  {
    // MY_ASSERT(InRange(sub));
    SetCellValue(sub.x(), sub.y(), sub.z(), value);
  }

  void SetCellValue(int index, const _T& value)
  {
    // MY_ASSERT(InRange(index));
    cells_[index] = value;
  }

private:
  Eigen::Vector3d origin_;
  Eigen::Vector3i size_;
  Eigen::Vector3d resolution_;
  Eigen::Vector3d resolution_inv_;
  std::vector<_T> cells_;
  std::vector<Eigen::Vector3i> subs_;
  int cell_number_;
  int dimension_;

  Eigen::Vector3i ind2sub_(int ind) const
  {
    // MY_ASSERT(InRange(ind));
    Eigen::Vector3i sub;
    sub.z() = ind / (size_.x() * size_.y());
    ind -= (sub.z() * size_.x() * size_.y());
    sub.y() = ind / size_.x();
    sub.x() = ind % size_.x();
    return sub;
  }
};
}  // namespace grid_ns