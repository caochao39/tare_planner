/**
 * @file lidar_model.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements the sensor model of a LiDAR
 * @version 0.1
 * @date 2019-09-26
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "lidar_model/lidar_model.h"

namespace lidar_model_ns
{
const double LiDARModel::kToDegreeConst = 57.2957;
const double LiDARModel::kToRadianConst = 0.01745;
const double LiDARModel::kEpsilon = 1e-4;
const double LiDARModel::kCloudInflateRatio = 2;
double LiDARModel::pointcloud_resolution_ = 0.2;

int LiDARModel::sub2ind(int row_index, int column_index) const
{
  return row_index * kHorizontalVoxelSize + column_index;
}

void LiDARModel::ind2sub(int ind, int& row_index, int& column_index) const
{
  row_index = ind / kHorizontalVoxelSize;
  column_index = ind % kHorizontalVoxelSize;
}

LiDARModel::LiDARModel(double px, double py, double pz, double rw, double rx, double ry, double rz)
{
  pose_.position.x = px;
  pose_.position.y = py;
  pose_.position.z = pz;
  pose_.orientation.w = rw;
  pose_.orientation.x = rx;
  pose_.orientation.y = ry;
  pose_.orientation.z = rz;
  ResetCoverage();
}

LiDARModel::LiDARModel(const geometry_msgs::Pose& pose)
  : LiDARModel(pose.position.x, pose.position.y, pose.position.z, pose.orientation.w, pose.orientation.x,
               pose.orientation.y, pose.orientation.z)
{
}

void LiDARModel::ResetCoverage()
{
  reset_.fill(true);
}

void LiDARModel::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& visualization_cloud, double resol,
                                       double max_range) const
{
  visualization_cloud->clear();
  geometry_msgs::Point start_point = pose_.position;
  for (int i = 0; i < covered_voxel_.size(); i++)
  {
    int row_index, column_index;
    ind2sub(i, row_index, column_index);
    double phi = (column_index * kHorizontalResolution - 180) * M_PI / 180;
    double theta = (row_index * kVerticalResolution - kVerticalAngleOffset) * M_PI / 180;

    double r = covered_voxel_[i];
    if (isZero(covered_voxel_[i]) || reset_[i])
    {
      r = max_range;
    }
    geometry_msgs::Point end_point;
    end_point.x = r * sin(theta) * cos(phi) + pose_.position.x;
    end_point.y = r * sin(theta) * sin(phi) + pose_.position.y;
    end_point.z = r * cos(theta) + pose_.position.z;
    pcl::PointXYZI point;
    point.x = end_point.x;
    point.y = end_point.y;
    point.z = end_point.z;
    point.intensity = 0.0;

    visualization_cloud->points.push_back(point);
    pcl::PointCloud<pcl::PointXYZI>::Ptr tmp_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    misc_utils_ns::LinInterpPoints<pcl::PointXYZI>(start_point, end_point, resol, tmp_cloud);
    for (auto& tmp_point : tmp_cloud->points)
    {
      if (isZero(covered_voxel_[i]) || reset_[i])
      {
        tmp_point.intensity = 0.0;
      }
      else
      {
        tmp_point.intensity = 10.0;
      }
    }
    *visualization_cloud += *tmp_cloud;
  }
}
}  // namespace lidar_model_ns
