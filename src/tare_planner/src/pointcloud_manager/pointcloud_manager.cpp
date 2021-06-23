/**
 * @file pointcloud_manager.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid of pointclouds
 * @version 0.1
 * @date 2019-12-08
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "../../include/pointcloud_manager/pointcloud_manager.h"

namespace pointcloud_manager_ns
{
PointCloudManager::PointCloudManager(int row_num, int col_num, int max_cell_point_num, double cell_size,
                                     int neighbor_cell_num)
  : kRowNum(row_num)
  , kColNum(col_num)
  , kMaxCellPointNum(max_cell_point_num)
  , kCellSize(cell_size)
  , kNeighborCellNum(neighbor_cell_num)
  , kCloudDwzFilterLeafSize(0.2)
  , initialized_(false)
{
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  origin_.x = robot_position_.x - (kCellSize * kRowNum) / 2;
  origin_.y = robot_position_.y - (kCellSize * kColNum) / 2;
  origin_.z = 0.0;

  cur_row_idx_ = kRowNum / 2;
  cur_col_idx_ = kColNum / 2;

  Eigen::Vector3i pointcloud_grid_size(kRowNum, kColNum, 1);
  Eigen::Vector3d pointcloud_grid_origin(origin_.x, origin_.y, origin_.z);
  Eigen::Vector3d pointcloud_grid_resolution(kCellSize, kCellSize, 100);  // TODO: make this 3D
  PCLCloudTypePtr cloud_ptr_tmp;
  pointcloud_grid_ = std::make_unique<grid_ns::Grid<PCLCloudTypePtr>>(
      pointcloud_grid_size, cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 2);

  pcl::PointCloud<pcl::PointXYZI>::Ptr occupancy_cloud_ptr_tmp;
  occupancy_cloud_grid_ = std::make_unique<grid_ns::Grid<pcl::PointCloud<pcl::PointXYZI>::Ptr>>(
      pointcloud_grid_size, occupancy_cloud_ptr_tmp, pointcloud_grid_origin, pointcloud_grid_resolution, 2);

  for (int i = 0; i < pointcloud_grid_->GetCellNumber(); i++)
  {
    pointcloud_grid_->GetCell(i) = PCLCloudTypePtr(new PCLCloudType);
    pointcloud_grid_->GetCell(i)->points.reserve(kMaxCellPointNum);
  }

  for (int i = 0; i < occupancy_cloud_grid_->GetCellNumber(); i++)
  {
    occupancy_cloud_grid_->GetCell(i) = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  }

  cloud_dwz_filter_.setLeafSize(kCloudDwzFilterLeafSize, kCloudDwzFilterLeafSize, kCloudDwzFilterLeafSize);
  rolled_in_occupancy_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

void PointCloudManager::UpdateOrigin()
{
  origin_.x = robot_position_.x - (kCellSize * kRowNum) / 2;
  origin_.y = robot_position_.y - (kCellSize * kColNum) / 2;
  origin_.z = 0.0;
  pointcloud_grid_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
  occupancy_cloud_grid_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
}

bool PointCloudManager::UpdateRobotPosition(const geometry_msgs::Point& robot_position)
{
  robot_position_ = robot_position;
  if (!initialized_)
  {
    initialized_ = true;
    UpdateOrigin();
  }

  Eigen::Vector3i robot_cell_sub =
      pointcloud_grid_->Pos2Sub(Eigen::Vector3d(robot_position_.x, robot_position_.y, robot_position_.z));

  // Get neighbor indices
  prev_neighbor_indices_ = neighbor_indices_;
  neighbor_indices_.clear();
  int row_idx;
  int col_idx;
  int N = kNeighborCellNum / 2;
  for (int i = -N; i <= N; i++)
  {
    for (int j = -N; j <= N; j++)
    {
      Eigen::Vector3i neighbor_sub;
      neighbor_sub.x() = robot_cell_sub.x() + i;
      neighbor_sub.y() = robot_cell_sub.y() + j;
      neighbor_sub.z() = 0;
      if (pointcloud_grid_->InRange(neighbor_sub))
      {
        int ind = pointcloud_grid_->Sub2Ind(neighbor_sub);
        neighbor_indices_.push_back(ind);
      }
    }
  }

  std::vector<int> indices_diff;
  misc_utils_ns::SetDifference(neighbor_indices_, prev_neighbor_indices_, indices_diff);
  bool rolling = false;
  if (!indices_diff.empty())
  {
    new_neighbor_indices_ = indices_diff;
    rolling = true;
  }

  Eigen::Vector3i neighbor_cell_min_sub = robot_cell_sub - Eigen::Vector3i(N, N, 0);
  neighbor_cells_origin_ =
      pointcloud_grid_->Sub2Pos(neighbor_cell_min_sub) - Eigen::Vector3d(kCellSize / 2, kCellSize / 2, 0);

  return rolling;
}

void PointCloudManager::GetPointCloud(PCLCloudType& cloud_out)
{
  cloud_out.clear();
  for (const auto& neighbor_ind : neighbor_indices_)
  {
    cloud_out += *(pointcloud_grid_->GetCell(neighbor_ind));
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudManager::GetRolledInOccupancyCloud()
{
  rolled_in_occupancy_cloud_->clear();
  for (const auto& ind : new_neighbor_indices_)
  {
    *rolled_in_occupancy_cloud_ += *(occupancy_cloud_grid_->GetCell(ind));
  }
  return rolled_in_occupancy_cloud_;
}

void PointCloudManager::StoreOccupancyCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& occupancy_cloud)
{
  for (const auto& point : occupancy_cloud->points)
  {
    Eigen::Vector3i sub = occupancy_cloud_grid_->Pos2Sub(Eigen::Vector3d(point.x, point.y, point.z));
    if (!occupancy_cloud_grid_->InRange(sub))
    {
      continue;
    }
    int ind = occupancy_cloud_grid_->Sub2Ind(sub);
    occupancy_cloud_grid_->GetCell(ind)->points.push_back(point);
  }
}

void PointCloudManager::GetMarker(visualization_msgs::Marker& marker)
{
  marker.points.clear();
  marker.colors.clear();
  marker.scale.x = kCellSize;
  marker.scale.y = kCellSize;
  marker.scale.z = 0.05;

  for (int i = 0; i < kRowNum; i++)
  {
    for (int j = 0; j < kColNum; j++)
    {
      geometry_msgs::Point cell_center;
      cell_center.x = i * kCellSize + kCellSize / 2 + origin_.x;
      cell_center.y = j * kCellSize + kCellSize / 2 + origin_.y;
      cell_center.z = 0.0;
      marker.points.push_back(cell_center);
      std_msgs::ColorRGBA color;
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 1.0;
      marker.colors.push_back(color);
    }
  }
  for (const auto& ind : neighbor_indices_)
  {
    marker.colors[ind].r = 0.0;
    marker.colors[ind].g = 1.0;
    marker.colors[ind].b = 0.0;
  }
}

void PointCloudManager::GetVisualizationPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud)
{
  vis_cloud->clear();
  for (const auto& ind : new_neighbor_indices_)
  {
    Eigen::Vector3d position = pointcloud_grid_->Ind2Pos(ind);
    pcl::PointXYZI point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    point.intensity = ind;
    vis_cloud->points.push_back(point);
  }
}

void PointCloudManager::GetCloudPointIndex(int index, int& cloud_index, int& cloud_point_index)
{
  cloud_index = -1;
  cloud_point_index = -1;
  int point_num = 0;
  for (int i = 0; i < neighbor_indices_.size(); i++)
  {
    int ind = neighbor_indices_[i];
    point_num += pointcloud_grid_->GetCell(ind)->points.size();

    if (index < point_num)
    {
      cloud_index = ind;
      int prev_point_num = point_num - pointcloud_grid_->GetCell(ind)->points.size();
      cloud_point_index = index - prev_point_num;
      break;
    }
  }
  if (index > point_num || cloud_index == -1 || cloud_point_index == -1)
  {
    std::cout << "index: " << index << " point num: " << point_num << std::endl;
    for (int i = 0; i < neighbor_indices_.size(); i++)
    {
      int ind = neighbor_indices_[i];
      std::cout << "cloud " << ind << " size: " << pointcloud_grid_->GetCell(ind)->points.size() << std::endl;
    }
  }
}
int PointCloudManager::GetAllPointNum()
{
  int num = 0;
  for (int i = 0; i < pointcloud_grid_->GetCellNumber(); ++i)
  {
    num += pointcloud_grid_->GetCell(i)->points.size();
  }
  return num;
}

void PointCloudManager::UpdateOldCloudPoints()
{
  for (int i = 0; i < pointcloud_grid_->GetCellNumber(); ++i)
  {
    int point_num = pointcloud_grid_->GetCell(i)->points.size();
    for (int j = 0; j < point_num; ++j)
    {
      pointcloud_grid_->GetCell(i)->points[j].r = 255;
    }
  }
}

void PointCloudManager::UpdateCoveredCloudPoints()
{
  for (int i = 0; i < pointcloud_grid_->GetCellNumber(); ++i)
  {
    int point_num = pointcloud_grid_->GetCell(i)->points.size();
    for (int j = 0; j < point_num; ++j)
    {
      if (pointcloud_grid_->GetCell(i)->points[j].g > 0)
      {
        pointcloud_grid_->GetCell(i)->points[j].g = 255;
      }
    }
  }
}

void PointCloudManager::UpdateCoveredCloudPoints(int cloud_index, int point_index)
{
  int cloud_num = pointcloud_grid_->GetCellNumber();
  MY_ASSERT(cloud_index >= 0 && cloud_index < cloud_num);
  int point_num = pointcloud_grid_->GetCell(cloud_index)->points.size();
  MY_ASSERT(point_index >= 0 && point_index < point_num);
  pointcloud_grid_->GetCell(cloud_index)->points[point_index].g = 255;
}

}  // namespace pointcloud_manager_ns
