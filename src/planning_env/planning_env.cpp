//
// Created by caochao on 06/03/20.
//

#include <planning_env/planning_env.h>
#include <viewpoint_manager/viewpoint_manager.h>

namespace planning_env_ns
{
void PlanningEnvParameters::ReadParameters(ros::NodeHandle& nh)
{
  kStackedCloudDwzLeafSize = misc_utils_ns::getParam<double>(nh, "kStackedCloudDwzLeafSize", 0.2);
  kPlannerCloudDwzLeafSize = misc_utils_ns::getParam<double>(nh, "kPlannerCloudDwzLeafSize", 0.2);
  kCollisionCloudDwzLeafSize = misc_utils_ns::getParam<double>(nh, "kCollisionCloudDwzLeafSize", 0.2);
  kCollisionCheckRadius = misc_utils_ns::getParam<double>(nh, "kCollisionCheckRadius", 0.4);
  kCollisionCheckPointNumThr = misc_utils_ns::getParam<int>(nh, "kCollisionCheckPointNumThr", 1);

  kKeyposeCloudStackNum = misc_utils_ns::getParam<int>(nh, "kKeyposeCloudStackNum", 5);
  kCoverageZMax = misc_utils_ns::getParam<double>(nh, "kCoverageZMax", 10.0);
  kCoverageZMin = misc_utils_ns::getParam<double>(nh, "kCoverageZMin", 1.0);

  kPointCloudRowNum = misc_utils_ns::getParam<int>(nh, "kPointCloudRowNum", 20);
  kPointCloudColNum = misc_utils_ns::getParam<int>(nh, "kPointCloudColNum", 20);
  kMaxCellPointNum = misc_utils_ns::getParam<int>(nh, "kMaxCellPointNum", 100000);
  kPointCloudCellSize = misc_utils_ns::getParam<double>(nh, "kPointCloudCellSize", 24.0);
  kPointCloudManagerNeighborCellNum = misc_utils_ns::getParam<int>(nh, "kPointCloudManagerNeighborCellNum", 5);
  kCoverCloudZSqueezeRatio = misc_utils_ns::getParam<double>(nh, "kCoverCloudZSqueezeRatio", 2.0);

  kUseFrontier = misc_utils_ns::getParam<bool>(nh, "kUseFrontier", false);
  kFrontierClusterTolerance = misc_utils_ns::getParam<double>(nh, "kFrontierClusterTolerance", 1.0);
  kFrontierClusterMinSize = misc_utils_ns::getParam<int>(nh, "kFrontierClusterMinSize", 30);
  kOccupancyGridOrigin.x() = misc_utils_ns::getParam<double>(nh, "kOccupancyGridOriginX", -100);
  kOccupancyGridOrigin.y() = misc_utils_ns::getParam<double>(nh, "kOccupancyGridOriginY", -100);
  kOccupancyGridOrigin.z() = misc_utils_ns::getParam<double>(nh, "kOccupancyGridOriginZ", -10);
  kOccupancyGridSize.x() = misc_utils_ns::getParam<double>(nh, "kOccupancyGridSizeX", 300);
  kOccupancyGridSize.y() = misc_utils_ns::getParam<double>(nh, "kOccupancyGridSizeY", 300);
  kOccupancyGridSize.z() = misc_utils_ns::getParam<double>(nh, "kOccupancyGridSizeZ", 30);
  kOccupancyGridResolution.x() = misc_utils_ns::getParam<double>(nh, "kOccupancyGridResolutionX", 0.3);
  kOccupancyGridResolution.y() = misc_utils_ns::getParam<double>(nh, "kOccupancyGridResolutionY", 0.3);
  kOccupancyGridResolution.z() = misc_utils_ns::getParam<double>(nh, "kOccupancyGridResolutionZ", 0.3);
  kExtractFrontierRange.x() = misc_utils_ns::getParam<double>(nh, "kExtractFrontierRangeX", 40);
  kExtractFrontierRange.y() = misc_utils_ns::getParam<double>(nh, "kExtractFrontierRangeY", 40);
  kExtractFrontierRange.z() = misc_utils_ns::getParam<double>(nh, "kExtractFrontierRangeZ", 3);
  kElminateFrontierWithLastKeypose = misc_utils_ns::getParam<bool>(nh, "kElminateFrontierWithLastKeypose", false);
}

PlanningEnv::PlanningEnv(ros::NodeHandle nh, ros::NodeHandle nh_private, std::string world_frame_id)
  : keypose_cloud_count_(0)
  , vertical_surface_extractor_()
  , vertical_frontier_extractor_()
  , robot_position_update_(false)
{
  pp_.ReadParameters(nh_private);
  keypose_cloud_stack_.resize(pp_.kKeyposeCloudStackNum);
  for (int i = 0; i < keypose_cloud_stack_.size(); i++)
  {
    keypose_cloud_stack_[i].reset(new pcl::PointCloud<PlannerCloudPointType>());
  }
  stacked_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "stacked_cloud", world_frame_id);
  stacked_cloud_kdtree_ = pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());
  coverage_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "coverage_cloud", world_frame_id);

  diff_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "diff_cloud", world_frame_id);

  collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  terrain_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "terrain_cloud", world_frame_id);

  planner_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(nh, "planner_cloud", world_frame_id);
  pointcloud_manager_ = std::make_unique<pointcloud_manager_ns::PointCloudManager>(
      pp_.kPointCloudRowNum, pp_.kPointCloudColNum, pp_.kMaxCellPointNum, pp_.kPointCloudCellSize,
      pp_.kPointCloudManagerNeighborCellNum);
  pointcloud_manager_->SetCloudDwzFilterLeafSize() = pp_.kPlannerCloudDwzLeafSize;

  squeezed_planner_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>>(
      nh, "squeezed_planner_cloud", world_frame_id);
  squeezed_planner_cloud_kdtree_ =
      pcl::KdTreeFLANN<PlannerCloudPointType>::Ptr(new pcl::KdTreeFLANN<PlannerCloudPointType>());

  uncovered_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_cloud", world_frame_id);
  uncovered_frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "uncovered_frontier_cloud", world_frame_id);
  frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "frontier_cloud", world_frame_id);
  filtered_frontier_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "filtered_frontier_cloud", world_frame_id);
  occupied_cloud_ =
      std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "occupied_cloud", world_frame_id);
  free_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "free_cloud", world_frame_id);
  unknown_cloud_ = std::make_unique<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>>(nh, "unknown_cloud", world_frame_id);

  occupancy_grid_ = std::make_unique<occupancy_grid_ns::OccupancyGrid>(pp_.kOccupancyGridOrigin, pp_.kOccupancyGridSize,
                                                                       pp_.kOccupancyGridResolution);
  // std::cout << "Initialized occupancy grid with:" << std::endl;
  // std::cout << "origin: " << pp_.kOccupancyGridOrigin.transpose() << std::endl;
  // std::cout << "size: " << pp_.kOccupancyGridSize.transpose() << std::endl;
  // std::cout << "resolution: " << pp_.kOccupancyGridResolution.transpose() << std::endl;
  // std::cout << "eliminate frontier with last keypose: " << pp_.kElminateFrontierWithLastKeypose << std::endl;

  kdtree_frontier_cloud_ = pcl::search::KdTree<pcl::PointXYZI>::Ptr(new pcl::search::KdTree<pcl::PointXYZI>);

  // TODO: Temporary
  if (pp_.kUseFrontier)
  {
    occupancy_grid_->SetExtractFrontierRange(pp_.kExtractFrontierRange);
  }

  // Todo: parameterize
  vertical_surface_extractor_.SetRadiusThreshold(0.2);
  vertical_surface_extractor_.SetZDiffMax(2.0);
  vertical_surface_extractor_.SetZDiffMin(pp_.kStackedCloudDwzLeafSize);
  vertical_frontier_extractor_.SetNeighborThreshold(2);

  double vertical_frontier_neighbor_search_radius =
      std::max(pp_.kOccupancyGridResolution.x(), pp_.kOccupancyGridResolution.y());
  vertical_frontier_neighbor_search_radius =
      std::max(vertical_frontier_neighbor_search_radius, pp_.kOccupancyGridResolution.z());
  vertical_frontier_extractor_.SetRadiusThreshold(vertical_frontier_neighbor_search_radius);
  double z_diff_max = vertical_frontier_neighbor_search_radius * 5;
  double z_diff_min = vertical_frontier_neighbor_search_radius;
  vertical_frontier_extractor_.SetZDiffMax(z_diff_max);
  vertical_frontier_extractor_.SetZDiffMin(z_diff_min);
  vertical_frontier_extractor_.SetNeighborThreshold(2);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PlanningEnv::GetCollisionCloud(bool inflate, double inflate_size)
{
  if (inflate && inflate_size > 0)
  {
    collision_cloud_->clear();
    // int infn = ceil(inflate_size / pp_.kStackedCloudDwzLeafSize);
    int infn = 1;
    double inflate_resol = inflate_size / infn;
    std::cout << "infn: " << infn << std::endl;
    for (int i = 0; i < stacked_cloud_->cloud_->points.size(); i++)
    {
      for (int x = -infn; x <= infn; x++)
      {
        for (int y = -infn; y <= infn; y++)
        {
          for (int z = -infn; z <= infn; z++)
          {
            pcl::PointXYZI point;
            point.x = stacked_cloud_->cloud_->points[i].x + x * inflate_resol;
            point.y = stacked_cloud_->cloud_->points[i].y + y * inflate_resol;
            point.z = stacked_cloud_->cloud_->points[i].z + z * inflate_resol;
            if (x == 0 && y == 0 && z == 0)
            {
              point.intensity = 0.0;
            }
            else
            {
              point.intensity = 1.0;
            }
            collision_cloud_->points.push_back(point);
          }
        }
      }
    }
    // Downsize
    std::cout << "stacked cloud size: " << stacked_cloud_->cloud_->points.size() << std::endl;
    std::cout << "inflated collision cloud size: " << collision_cloud_->points.size() << std::endl;
    collision_cloud_downsizer_.Downsize(collision_cloud_, pp_.kStackedCloudDwzLeafSize, pp_.kStackedCloudDwzLeafSize,
                                        pp_.kStackedCloudDwzLeafSize);
    std::cout << "downsized collision cloud size: " << collision_cloud_->points.size() << std::endl;
  }
  else
  {
    // pcl::copyPointCloud(*(stacked_cloud_->cloud_), *collision_cloud_);
  }
  return collision_cloud_;
}

void PlanningEnv::UpdateTerrainCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  if (cloud->points.empty())
  {
    ROS_WARN("Terrain cloud empty");
  }
  else
  {
    terrain_cloud_->cloud_ = cloud;
  }
}

bool PlanningEnv::InCollision(double x, double y, double z) const
{
  if (stacked_cloud_->cloud_->points.empty())
  {
    ROS_WARN("PlanningEnv::InCollision(): collision cloud empty, not checking collision");
    return false;
  }
  PlannerCloudPointType check_point;
  check_point.x = x;
  check_point.y = y;
  check_point.z = z;
  std::vector<int> neighbor_indices;
  std::vector<float> neighbor_sqdist;
  stacked_cloud_kdtree_->radiusSearch(check_point, pp_.kCollisionCheckRadius, neighbor_indices, neighbor_sqdist);
  if (neighbor_indices.size() > pp_.kCollisionCheckPointNumThr)
  {
    return true;
  }
  else
  {
    return false;
  }
}

void PlanningEnv::UpdateCoveredArea(const lidar_model_ns::LiDARModel& robot_viewpoint,
                                    const std::unique_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  if (planner_cloud_->cloud_->points.empty())
  {
    std::cout << "Planning cloud empty, cannot update covered area" << std::endl;
    return;
  }
  // misc_utils_ns::Timer timer1("get covered area");
  // timer1.Start();
  geometry_msgs::Point robot_position = robot_viewpoint.getPosition();
  double sensor_range = viewpoint_manager->GetSensorRange();
  double coverage_occlusion_thr = viewpoint_manager->GetCoverageOcclusionThr();
  double coverage_dilation_radius = viewpoint_manager->GetCoverageDilationRadius();
  std::vector<int> covered_point_indices;
  // double vertical_fov_ratio = tan(M_PI / 12);
  double vertical_fov_ratio = 0.3;  // bigger fov than viewpoints
  double diff_z_max = sensor_range * vertical_fov_ratio;
  double xy_dist_threshold = 3 * (pp_.kPlannerCloudDwzLeafSize / 2) / 0.3;
  double z_diff_threshold = 3 * pp_.kPlannerCloudDwzLeafSize;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      planner_cloud_->cloud_->points[i].g = 255;
      // covered_point_indices.push_back(i);
      continue;
    }
    if (std::abs(point.z - robot_position.z) < diff_z_max)
    {
      if (misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                     Eigen::Vector3d(robot_position.x, robot_position.y, robot_position.z),
                                     vertical_fov_ratio, sensor_range, xy_dist_threshold, z_diff_threshold))
      {
        if (robot_viewpoint.CheckVisibility<PlannerCloudPointType>(point, coverage_occlusion_thr))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          continue;
        }
      }
    }
    // mark covered by visited viewpoints
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          planner_cloud_->cloud_->points[i].g = 255;
          covered_point_indices.push_back(i);
          break;
        }
      }
    }
  }
  // timer1.Stop(true);

  // Dilate the covered area
  // misc_utils_ns::Timer timer2("dilate covered area");
  // timer2.Start();
  squeezed_planner_cloud_->cloud_->clear();
  for (const auto& point : planner_cloud_->cloud_->points)
  {
    PlannerCloudPointType squeezed_point = point;
    squeezed_point.z = point.z / pp_.kCoverCloudZSqueezeRatio;
    squeezed_planner_cloud_->cloud_->points.push_back(squeezed_point);
  }
  squeezed_planner_cloud_kdtree_->setInputCloud(squeezed_planner_cloud_->cloud_);

  for (const auto& ind : covered_point_indices)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[ind];
    std::vector<int> nearby_indices;
    std::vector<float> nearby_sqdist;
    squeezed_planner_cloud_kdtree_->radiusSearch(point, coverage_dilation_radius, nearby_indices, nearby_sqdist);
    if (!nearby_indices.empty())
    {
      for (const auto& idx : nearby_indices)
      {
        MY_ASSERT(idx >= 0 && idx < planner_cloud_->cloud_->points.size());
        planner_cloud_->cloud_->points[idx].g = 255;
      }
    }
  }
  // timer2.Stop(true);

  // Update covered pointcloud in pointcloud_manager_
  // misc_utils_ns::Timer timer3("feedback pointcloud_manager");
  // timer3.Start();
  // int cloud_num = pointcloud_manager_->clouds_.size();
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      int cloud_idx = 0;
      int cloud_point_idx = 0;
      pointcloud_manager_->GetCloudPointIndex(i, cloud_idx, cloud_point_idx);
      pointcloud_manager_->UpdateCoveredCloudPoints(cloud_idx, cloud_point_idx);
      // MY_ASSERT(cloud_idx >= 0 && cloud_idx < cloud_num && cloud_point_idx >= 0 &&
      //           cloud_point_idx < pointcloud_manager_->clouds_[cloud_idx]->points.size());
      // pointcloud_manager_->clouds_[cloud_idx]->points[cloud_point_idx].g = 255;
    }
  }
  // timer3.Stop(true);
}

void PlanningEnv::GetUncoveredArea(const std::unique_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                   int& uncovered_point_num, int& uncovered_frontier_point_num)
{
  // misc_utils_ns::Timer timer1("get uncovered area");
  // timer1.Start();
  // Clear viewpoint covered point list
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    viewpoint_manager->ResetViewPointCoveredPointList(viewpoint_ind);
  }

  // Get uncovered points
  uncovered_cloud_->cloud_->clear();
  uncovered_frontier_cloud_->cloud_->clear();
  uncovered_point_num = 0;
  uncovered_frontier_point_num = 0;
  // std::cout << "planner cloud size: " << planner_cloud_->cloud_->points.size() << std::endl;
  for (int i = 0; i < planner_cloud_->cloud_->points.size(); i++)
  {
    PlannerCloudPointType point = planner_cloud_->cloud_->points[i];
    if (point.g > 0)
    {
      continue;
    }
    bool observed = false;
    for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
    {
      if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        if (viewpoint_manager->VisibleByViewPoint<PlannerCloudPointType>(point, viewpoint_ind))
        {
          viewpoint_manager->AddUncoveredPoint(viewpoint_ind, uncovered_point_num);
          observed = true;
        }
      }
    }
    if (observed)
    {
      pcl::PointXYZI uncovered_point;
      uncovered_point.x = point.x;
      uncovered_point.y = point.y;
      uncovered_point.z = point.z;
      uncovered_point.intensity = i;
      uncovered_cloud_->cloud_->points.push_back(uncovered_point);
      uncovered_point_num++;
    }
  }

  // Check uncovered frontiers
  if (pp_.kUseFrontier)
  {
    for (int i = 0; i < filtered_frontier_cloud_->cloud_->points.size(); i++)
    {
      pcl::PointXYZI point = filtered_frontier_cloud_->cloud_->points[i];
      bool observed = false;
      for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
      {
        if (!viewpoint_manager->ViewPointVisited(viewpoint_ind))
        {
          if (viewpoint_manager->VisibleByViewPoint<pcl::PointXYZI>(point, viewpoint_ind))
          {
            viewpoint_manager->AddUncoveredFrontierPoint(viewpoint_ind, uncovered_frontier_point_num);
            observed = true;
          }
        }
      }
      if (observed)
      {
        pcl::PointXYZI uncovered_frontier_point;
        uncovered_frontier_point.x = point.x;
        uncovered_frontier_point.y = point.y;
        uncovered_frontier_point.z = point.z;
        uncovered_frontier_point.intensity = i;
        uncovered_frontier_cloud_->cloud_->points.push_back(uncovered_frontier_point);
        uncovered_frontier_point_num++;
      }
    }
  }

  // timer1.Stop(true);
  // std::cout << "uncovered_point_num: " << uncovered_point_num << " unvisited viewpoint num: " < < < < std::endl;
}
void PlanningEnv::PublishStackedCloud()
{
  stacked_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredCloud()
{
  uncovered_cloud_->Publish();
}

void PlanningEnv::PublishUncoveredFrontierCloud()
{
  uncovered_frontier_cloud_->Publish();
}

}  // namespace planning_env_ns