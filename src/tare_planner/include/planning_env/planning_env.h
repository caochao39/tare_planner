//
// Created by caochao on 06/04/20.
//

#ifndef PLANNING_ENV_H
#define PLANNING_ENV_H

#include <cmath>
#include <vector>
#include <memory>
#include <Eigen/Core>
// ROS
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Polygon.h>
// PCL
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/PointIndices.h>

// Third parties
#include <utils/pointcloud_utils.h>
// Components
#include <pointcloud_manager/pointcloud_manager.h>
#include <lidar_model/lidar_model.h>
#include <occupancy_grid/occupancy_grid.h>

namespace viewpoint_manager_ns
{
class ViewPointManager;
}

namespace planning_env_ns
{
typedef pcl::PointXYZRGBNormal PlannerCloudPointType;
typedef pcl::PointCloud<PlannerCloudPointType> PlannerCloudType;
struct PlanningEnvParameters;
class PlanningEnv;
}  // namespace planning_env_ns

struct planning_env_ns::PlanningEnvParameters
{
  // Collision check
  double kStackedCloudDwzLeafSize;
  double kPlannerCloudDwzLeafSize;
  double kCollisionCloudDwzLeafSize;
  double kCollisionCheckRadius;
  int kCollisionCheckPointNumThr;

  int kKeyposeCloudStackNum;

  double kCoverageZMax;
  double kCoverageZMin;

  int kPointCloudRowNum;
  int kPointCloudColNum;
  int kMaxCellPointNum;
  double kPointCloudCellSize;
  int kPointCloudManagerNeighborCellNum;
  double kCoverCloudZSqueezeRatio;

  // Occupancy Grid
  bool kUseFrontier;
  double kFrontierClusterTolerance;
  int kFrontierClusterMinSize;
  Eigen::Vector3d kOccupancyGridOrigin;
  Eigen::Vector3d kOccupancyGridSize;
  Eigen::Vector3d kOccupancyGridResolution;
  Eigen::Vector3d kExtractFrontierRange;
  bool kElminateFrontierWithLastKeypose;

  void ReadParameters(ros::NodeHandle& nh);
};

class planning_env_ns::PlanningEnv
{
private:
  PlanningEnvParameters pp_;

  std::vector<typename PlannerCloudType::Ptr> keypose_cloud_stack_;
  int keypose_cloud_count_;
  Eigen::Vector3d robot_position_;
  Eigen::Vector3d prev_robot_position_;
  bool robot_position_update_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> stacked_cloud_;
  pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr stacked_cloud_kdtree_;
  pointcloud_utils_ns::PointCloudDownsizer<PlannerCloudPointType> stacked_cloud_downsizer_;
  pointcloud_utils_ns::PointCloudDownsizer<pcl::PointXYZI> collision_cloud_downsizer_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> coverage_cloud_;
  pointcloud_utils_ns::VerticalSurfaceExtractor vertical_surface_extractor_;
  pointcloud_utils_ns::VerticalSurfaceExtractor vertical_frontier_extractor_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> diff_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_cloud_;

  geometry_msgs::Polygon coverage_boundary_;

  std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> planner_cloud_;
  std::unique_ptr<pointcloud_manager_ns::PointCloudManager> pointcloud_manager_;

  std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> squeezed_planner_cloud_;
  pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr squeezed_planner_cloud_kdtree_;

  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> uncovered_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> uncovered_frontier_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> frontier_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> filtered_frontier_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> occupied_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> free_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> unknown_cloud_;

  std::unique_ptr<occupancy_grid_ns::OccupancyGrid> occupancy_grid_;

  pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree_frontier_cloud_;

public:
  PlanningEnv(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string world_frame_id = "/map");
  ~PlanningEnv() = default;
  double GetPlannerCloudResolution()
  {
    return pp_.kPlannerCloudDwzLeafSize;
  }
  void UpdateRobotPosition(geometry_msgs::Point robot_position)
  {
    pointcloud_manager_->UpdateRobotPosition(robot_position);
    robot_position_.x() = robot_position.x;
    robot_position_.y() = robot_position.y;
    robot_position_.z() = robot_position.z;
    if (!robot_position_update_)
    {
      prev_robot_position_ = robot_position_;
    }
    robot_position_update_ = true;
  }
  template <class PCLPointType>
  void UpdateRegisteredCloud(typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    if (cloud->points.empty())
    {
      ROS_WARN("PlanningEnv::UpdateRegisteredCloud(): registered cloud empty");
      return;
    }
    else
    {
      if (pp_.kUseFrontier)
      {
        occupancy_grid_->UpdateOccupancy<PCLPointType>(cloud);
        occupancy_grid_->RayTrace(robot_position_);
        occupied_cloud_->cloud_->clear();
        free_cloud_->cloud_->clear();
        unknown_cloud_->cloud_->clear();
        occupancy_grid_->GetVisualizationCloudInRange(robot_position_, pp_.kExtractFrontierRange,
                                                      occupied_cloud_->cloud_, free_cloud_->cloud_,
                                                      unknown_cloud_->cloud_);

        // occupied_cloud_->Publish();
        // free_cloud_->Publish();
        // unknown_cloud_->Publish();
      }
    }
  }

  template <class PCLPointType>
  void UpdateKeyposeCloud(typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    if (cloud->points.empty())
    {
      ROS_WARN("PlanningEnv::UpdateKeyposeCloud(): keypose cloud empty");
      return;
    }
    else
    {
      // Extract surface of interest
      misc_utils_ns::Timer get_surface_timer("get coverage and diff cloud");
      get_surface_timer.Start();
      coverage_cloud_->cloud_->clear();

      vertical_surface_extractor_.ExtractVerticalSurface<PCLPointType, PlannerCloudPointType>(cloud,
                                                                                              coverage_cloud_->cloud_);
      // coverage_cloud_->Publish();

      // Get the diff cloud
      diff_cloud_->cloud_->clear();
      for (auto& point : coverage_cloud_->cloud_->points)
      {
        point.r = 0;
        point.g = 0;
        point.b = 0;
      }
      // for (auto& cloud : pointcloud_manager_->clouds_)
      // {
      //   for (auto& point : cloud->points)
      //   {
      //     point.r = 255;
      //   }
      // }
      pointcloud_manager_->UpdateOldCloudPoints();

      planner_cloud_->cloud_->clear();
      pointcloud_manager_->UpdatePointCloud<PlannerCloudPointType>(*(coverage_cloud_->cloud_));
      // Update covered
      // for (int i = 0; i < pointcloud_manager_->clouds_.size(); i++)
      // {
      //   for (int j = 0; j < pointcloud_manager_->clouds_[i]->points.size(); j++)
      //   {
      //     if (pointcloud_manager_->clouds_[i]->points[j].g > 0)
      //     {
      //       pointcloud_manager_->clouds_[i]->points[j].g = 255;
      //     }
      //   }
      // }
      pointcloud_manager_->UpdateCoveredCloudPoints();
      pointcloud_manager_->GetPointCloud(*(planner_cloud_->cloud_));
      planner_cloud_->Publish();

      for (auto& point : stacked_cloud_->cloud_->points)
      {
        point.r = 255;
      }
      *(stacked_cloud_->cloud_) += *(coverage_cloud_->cloud_);
      stacked_cloud_downsizer_.Downsize(stacked_cloud_->cloud_, pp_.kStackedCloudDwzLeafSize,
                                        pp_.kStackedCloudDwzLeafSize, pp_.kStackedCloudDwzLeafSize);
      for (const auto& point : stacked_cloud_->cloud_->points)
      {
        if (point.r < 40)  // TODO: computed from the keypose cloud resolution and stacked cloud resolution
        {
          diff_cloud_->cloud_->points.push_back(point);
        }
      }
      diff_cloud_->Publish();
      get_surface_timer.Stop(false);

      // Stack together
      keypose_cloud_stack_[keypose_cloud_count_]->clear();
      // *keypose_cloud_stack_[keypose_cloud_count_] = *cloud;
      *keypose_cloud_stack_[keypose_cloud_count_] = *(coverage_cloud_->cloud_);
      keypose_cloud_count_ = (keypose_cloud_count_ + 1) % pp_.kKeyposeCloudStackNum;
      stacked_cloud_->cloud_->clear();
      for (int i = 0; i < pp_.kKeyposeCloudStackNum; i++)
      {
        *(stacked_cloud_->cloud_) += *keypose_cloud_stack_[i];
      }

      stacked_cloud_downsizer_.Downsize(stacked_cloud_->cloud_, pp_.kStackedCloudDwzLeafSize,
                                        pp_.kStackedCloudDwzLeafSize, pp_.kStackedCloudDwzLeafSize);
      stacked_cloud_kdtree_->setInputCloud(stacked_cloud_->cloud_);

      collision_cloud_->clear();
      for (int i = 0; i < pp_.kKeyposeCloudStackNum; i++)
      {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::copyPointCloud<PlannerCloudPointType, pcl::PointXYZI>(*keypose_cloud_stack_[i], *cloud_tmp);
        *(collision_cloud_) += *cloud_tmp;
      }
      collision_cloud_downsizer_.Downsize(collision_cloud_, pp_.kCollisionCloudDwzLeafSize,
                                          pp_.kCollisionCloudDwzLeafSize, pp_.kCollisionCloudDwzLeafSize);

      if (pp_.kUseFrontier)
      {
        // misc_utils_ns::Timer get_frontier_timer("get frontier");
        // get_frontier_timer.Start();
        // occupancy_grid_->UpdateOccupancy<PCLPointType>(cloud);
        // frontier_cloud_->cloud_->clear();
        // occupancy_grid_->RayTrace(robot_position_);
        // occupancy_grid_->GetFrontierInRange(frontier_cloud_->cloud_, robot_position_);
        // get_frontier_timer.Stop(true);

        // occupied_cloud_->cloud_->clear();
        // free_cloud_->cloud_->clear();
        // unknown_cloud_->cloud_->clear();
        // occupancy_grid_->GetVisualizationCloudInRange(robot_position_, pp_.kExtractFrontierRange,
        //                                               occupied_cloud_->cloud_, free_cloud_->cloud_,
        //                                               unknown_cloud_->cloud_);
        // vertical_frontier_extractor_.ExtractVerticalSurface<pcl::PointXYZI, pcl::PointXYZI>(
        //     frontier_cloud_->cloud_, filtered_frontier_cloud_->cloud_);

        // occupied_cloud_->Publish();
        // free_cloud_->Publish();
        // unknown_cloud_->Publish();
        // frontier_cloud_->Publish();
        // filtered_frontier_cloud_->Publish();
        if (pp_.kElminateFrontierWithLastKeypose && robot_position_update_)
        {
          occupancy_grid_->SetEliminateFrontier(true);
        }
        else
        {
          occupancy_grid_->SetEliminateFrontier(false);
        }

        occupancy_grid_->SetEliminateFrontierOrigin(prev_robot_position_);
        occupancy_grid_->GetFrontierInRange(frontier_cloud_->cloud_, robot_position_);
        prev_robot_position_ = robot_position_;

        if (!frontier_cloud_->cloud_->points.empty())
        {
          vertical_frontier_extractor_.ExtractVerticalSurface<pcl::PointXYZI, pcl::PointXYZI>(
              frontier_cloud_->cloud_, filtered_frontier_cloud_->cloud_);
        }
        // frontier_cloud_->Publish();

        // Cluster frontiers
        if (!filtered_frontier_cloud_->cloud_->points.empty())
        {
          kdtree_frontier_cloud_->setInputCloud(filtered_frontier_cloud_->cloud_);
          std::vector<pcl::PointIndices> cluster_indices;
          pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
          ec.setClusterTolerance(pp_.kFrontierClusterTolerance);
          ec.setMinClusterSize(1);
          ec.setMaxClusterSize(10000);
          ec.setSearchMethod(kdtree_frontier_cloud_);
          ec.setInputCloud(filtered_frontier_cloud_->cloud_);
          ec.extract(cluster_indices);

          pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
          int cluster_count = 0;
          for (int i = 0; i < cluster_indices.size(); i++)
          {
            if (cluster_indices[i].indices.size() < pp_.kFrontierClusterMinSize)
            {
              continue;
            }
            for (int j = 0; j < cluster_indices[i].indices.size(); j++)
            {
              int point_ind = cluster_indices[i].indices[j];
              filtered_frontier_cloud_->cloud_->points[point_ind].intensity = cluster_count;
              inliers->indices.push_back(point_ind);
            }
            cluster_count++;
          }
          pcl::ExtractIndices<pcl::PointXYZI> extract;
          extract.setInputCloud(filtered_frontier_cloud_->cloud_);
          extract.setIndices(inliers);
          extract.setNegative(false);
          extract.filter(*(filtered_frontier_cloud_->cloud_));
          filtered_frontier_cloud_->Publish();
        }
      }
    }
  }
  inline void UpdateCoverageBoundary(const geometry_msgs::Polygon& polygon)
  {
    coverage_boundary_ = polygon;
  }
  template <class PCLPointType>
  void GetCoverageCloudWithinBoundary(typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    for (const auto& point : cloud->points)
    {
      geometry_msgs::Point geo_point;
      geo_point.x = point.x;
      geo_point.y = point.y;
      geo_point.z = point.z;
      if (point.z >= pp_.kCoverageZMin && point.z <= pp_.kCoverageZMax &&
          misc_utils_ns::PointInPolygon(geo_point, coverage_boundary_))
      {
        PlannerCloudPointType coverage_cloud_point;
        coverage_cloud_point.x = point.x;
        coverage_cloud_point.y = point.y;
        coverage_cloud_point.z = point.z;
        coverage_cloud_->cloud_->points.push_back(coverage_cloud_point);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr GetCollisionCloud(bool inflate = false, double inflate_size = 0.0);
  pcl::PointCloud<PlannerCloudPointType>::Ptr GetStackedCloud()
  {
    return stacked_cloud_->cloud_;
  }

  void UpdateTerrainCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void UpdateCollisionCostGrid();
  bool InCollision(double x, double y, double z) const;

  inline pcl::PointCloud<PlannerCloudPointType>::Ptr GetDiffCloud()
  {
    return diff_cloud_->cloud_;
  }
  inline pcl::PointCloud<PlannerCloudPointType>::Ptr GetPlannerCloud()
  {
    return planner_cloud_->cloud_;
  }
  void UpdateCoveredArea(const lidar_model_ns::LiDARModel& robot_viewpoint,
                         const std::unique_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);

  void GetUncoveredArea(const std::unique_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                        int& uncovered_point_num, int& uncovered_frontier_point_num);

  void PublishStackedCloud();
  void PublishUncoveredCloud();
  void PublishUncoveredFrontierCloud();
};

#endif  // PLANNING_ENV_H
