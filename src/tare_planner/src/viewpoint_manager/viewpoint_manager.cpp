/**
 * @file viewpoint_manager.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the viewpoints inside the local planning horizon
 * @version 0.1
 * @date 2020-06-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#include "viewpoint_manager/viewpoint_manager.h"

namespace viewpoint_manager_ns
{
const std::string ViewPointManager::kRuntimeUnit = "us";

bool ViewPointManagerParameter::ReadParameters(ros::NodeHandle& nh)
{
  kUseFrontier = misc_utils_ns::getParam<bool>(nh, "kUseFrontier", false);

  dimension_ = 2;

  kNum.x() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/size_x", 80);
  kNum.y() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/size_y", 80);
  kNum.z() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/size_z", 40);
  kViewPointNum = kNum.x() * kNum.y() * kNum.z();
  kRolloverStepsize.x() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/rollover_step_size_x", 16);
  kRolloverStepsize.y() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/rollover_step_size_y", 16);
  kRolloverStepsize.z() = misc_utils_ns::getParam<int>(nh, "viewpoint_manager/rollover_step_size_z", 10);

  kResol.x() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resol_x", 0.5);
  kResol.y() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resol_y", 0.5);
  kResol.z() = misc_utils_ns::getParam<double>(nh, "viewpoint_manager/resol_z", 0.5);
  kCollisionCheckTerrainThr = misc_utils_ns::getParam<double>(nh, "kCollisionCheckTerrainThr", 0.25);
  kViewPointZMax = misc_utils_ns::getParam<double>(nh, "kViewPointZMax", 8.0);
  kViewPointZMin = misc_utils_ns::getParam<double>(nh, "kViewPointZMin", 1.0);
  kCollisionRadius = misc_utils_ns::getParam<double>(nh, "kCollisionRadius", 5.0);
  kCollisionGridZScale = misc_utils_ns::getParam<double>(nh, "kCollisionGridZScale", 2.0);
  kCollisionGridResolution.x() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionX", 0.5);
  kCollisionGridResolution.y() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionY", 0.5);
  kCollisionGridResolution.z() = misc_utils_ns::getParam<double>(nh, "kCollisionGridResolutionZ", 0.5);
  kLineOfSightStopAtNearestObstacle = misc_utils_ns::getParam<bool>(nh, "kLineOfSightStopAtNearestObstacle", true);
  kCheckDynamicObstacleCollision = misc_utils_ns::getParam<bool>(nh, "kCheckDynamicObstacleCollision", true);
  kCollisionFrameCountMax = misc_utils_ns::getParam<int>(nh, "kCollisionFrameCountMax", 3);
  kViewPointHeightFromTerrain = misc_utils_ns::getParam<double>(nh, "kViewPointHeightFromTerrain", 0.75);
  kViewPointHeightFromTerrainChangeThreshold =
      misc_utils_ns::getParam<double>(nh, "kViewPointHeightFromTerrainChangeThreshold", 0.6);

  kCollisionPointThr = misc_utils_ns::getParam<int>(nh, "kCollisionPointThr", 3);

  for (int i = 0; i < dimension_; i++)
  {
    LocalPlanningHorizonSize(i) = kNum(i) * kResol(i);
  }

  kCollisionGridSize = Eigen::Vector3i::Ones();
  for (int i = 0; i < dimension_; i++)
  {
    kCollisionGridSize(i) = ceil((kNum(i) * kResol(i) + kCollisionRadius * 2) / kCollisionGridResolution(i));
  }

  kCoverageOcclusionThr = misc_utils_ns::getParam<double>(nh, "kCoverageOcclusionThr", 1.0);
  kCoverageDilationRadius = misc_utils_ns::getParam<double>(nh, "kCoverageDilationRadius", 1.0);
  kCoveragePointCloudResolution = misc_utils_ns::getParam<double>(nh, "kPlannerCloudDwzLeafSize", 1.0);
  kMinAddPointNum = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumSmall", 60);
  kMinAddFrontierPointNum = misc_utils_ns::getParam<int>(nh, "kMinAddFrontierPointNum", 30);
  kGreedyViewPointSampleRange = misc_utils_ns::getParam<int>(nh, "kGreedyViewPointSampleRange", 5);
  kLocalPathOptimizationItrMax = misc_utils_ns::getParam<int>(nh, "kLocalPathOptimizationItrMax", 10);

  kSensorRange = misc_utils_ns::getParam<double>(nh, "kSensorRange", 10.0);
  kNeighborRange = misc_utils_ns::getParam<double>(nh, "kNeighborRange", 3.0);

  kVerticalFOVRatio = tan(M_PI / 15);
  kDiffZMax = kSensorRange * kVerticalFOVRatio;
  kInFovXYDistThreshold = 3 * (kCoveragePointCloudResolution / 2) / tan(M_PI / 15);
  kInFovZDiffThreshold = 3 * kCoveragePointCloudResolution;

  return true;
}

ViewPointManager::ViewPointManager(ros::NodeHandle& nh)
  : initialized_(false), lookahead_point_update_(false), local_coverage_complete_(false)
{
  vp_.ReadParameters(nh);

  kdtree_viewpoint_candidate_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_viewpoint_in_collision_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  viewpoint_candidate_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  viewpoint_in_collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  grid_ = std::make_unique<rollable_grid_ns::RollableGrid>(vp_.kNum);
  origin_ = Eigen::Vector3d::Zero();

  viewpoints_.resize(vp_.kViewPointNum);
  for (int x = 0; x < vp_.kNum.x(); x++)
  {
    for (int y = 0; y < vp_.kNum.y(); y++)
    {
      for (int z = 0; z < vp_.kNum.z(); z++)
      {
        Eigen::Vector3i sub(x, y, z);
        int ind = grid_->Sub2Ind(sub);
        viewpoints_[ind] = viewpoint_ns::ViewPoint();
      }
    }
  }

  graph_index_map_.resize(vp_.kViewPointNum);
  for (auto& ind : graph_index_map_)
  {
    ind = -1;
  }

  for (int i = 0; i < vp_.kViewPointNum; i++)
  {
    std::vector<int> array_indices;
    binding_viewpoint_array_indices_.push_back(array_indices);
    bound_viewpoint_array_ind_.push_back(-1);
  }

  ComputeConnectedNeighborIndices();
  ComputeInRangeNeighborIndices();
  GetCollisionCorrespondence();

  local_planning_horizon_size_ = Eigen::Vector3d::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    local_planning_horizon_size_(i) = vp_.kNum(i) * vp_.kResol(i);
  }
}

void ViewPointManager::ComputeConnectedNeighborIndices()
{
  connected_neighbor_indices_.resize(vp_.kViewPointNum);
  connected_neighbor_dist_.resize(vp_.kViewPointNum);

  std::vector<Eigen::Vector3i> idx_addon;
  for (int x = -1; x <= 1; x++)
  {
    for (int y = -1; y <= 1; y++)
    {
      for (int z = -1; z <= 1; z++)
      {
        if (x == 0 && y == 0 && z == 0)
          continue;
        idx_addon.push_back(Eigen::Vector3i(x, y, z));
      }
    }
  }

  for (int x = 0; x < vp_.kNum.x(); x++)
  {
    for (int y = 0; y < vp_.kNum.y(); y++)
    {
      for (int z = 0; z < vp_.kNum.z(); z++)
      {
        Eigen::Vector3i sub(x, y, z);
        int ind = grid_->Sub2Ind(sub);
        for (int i = 0; i < idx_addon.size(); i++)
        {
          Eigen::Vector3i neighbor_sub = sub + idx_addon[i];
          if (grid_->InRange(neighbor_sub))
          {
            connected_neighbor_indices_[ind].push_back(grid_->Sub2Ind(neighbor_sub));
            double dist = sqrt(vp_.kResol.x() * vp_.kResol.x() * std::abs(idx_addon[i].x()) +
                               vp_.kResol.y() * vp_.kResol.y() * std::abs(idx_addon[i].y()) +
                               vp_.kResol.z() * vp_.kResol.z() * std::abs(idx_addon[i].z()));
            connected_neighbor_dist_[ind].push_back(dist);
          }
        }
      }
    }
  }
}

void ViewPointManager::ComputeInRangeNeighborIndices()
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < vp_.kViewPointNum; i++)
  {
    Eigen::Vector3i sub = grid_->Ind2Sub(i);
    pcl::PointXYZ point;
    point.x = sub.x() * vp_.kResol.x() + vp_.kResol.x() / 2.0;
    point.y = sub.y() * vp_.kResol.y() + vp_.kResol.y() / 2.0;
    point.z = sub.z() * vp_.kResol.z() + vp_.kResol.z() / 2.0;
    cloud->points.push_back(point);
  }

  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree =
      pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZ>());
  kdtree->setInputCloud(cloud);
  in_range_neighbor_indices_.resize(vp_.kViewPointNum);
  std::vector<int> in_range_indices;
  std::vector<float> in_range_sqdist;
  for (int i = 0; i < in_range_neighbor_indices_.size(); i++)
  {
    pcl::PointXYZ point = cloud->points[i];
    kdtree->radiusSearch(point, vp_.kNeighborRange, in_range_indices, in_range_sqdist);
    for (const auto& ind : in_range_indices)
    {
      in_range_neighbor_indices_[i].push_back(ind);
    }
  }
}

int ViewPointManager::GetViewPointArrayInd(int viewpoint_ind, bool use_array_ind) const
{
  MY_ASSERT(grid_->InRange(viewpoint_ind));
  return (use_array_ind ? viewpoint_ind : grid_->GetArrayInd(viewpoint_ind));
}

void ViewPointManager::GetCollisionCorrespondence()
{
  misc_utils_ns::Timer timer("get collision grid correspondence");
  timer.Start();

  collision_grid_origin_ = Eigen::Vector3d::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    collision_grid_origin_(i) -= vp_.kCollisionRadius;
  }
  std::vector<int> viewpoint_index_correspondence;
  collision_grid_ = std::make_unique<grid_ns::Grid<std::vector<int>>>(
      vp_.kCollisionGridSize, viewpoint_index_correspondence, collision_grid_origin_, vp_.kCollisionGridResolution, 2);
  collision_point_count_.resize(collision_grid_->GetCellNumber(), 0);

  pcl::PointCloud<pcl::PointXYZI>::Ptr viewpoint_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree(new pcl::KdTreeFLANN<pcl::PointXYZI>());

  // Get viewpoint cloud
  // int count = 0;
  for (int x = 0; x < vp_.kNum.x(); x++)
  {
    for (int y = 0; y < vp_.kNum.y(); y++)
    {
      for (int z = 0; z < vp_.kNum.z(); z++)
      {
        int ind = grid_->Sub2Ind(Eigen::Vector3i(x, y, z));
        pcl::PointXYZI point;
        point.x = (x + 0.5) * vp_.kResol.x();
        point.y = (y + 0.5) * vp_.kResol.y();
        point.z = (z + 0.5) * vp_.kResol.z();
        point.z *= vp_.kCollisionGridZScale;
        point.intensity = ind;
        viewpoint_cloud->points.push_back(point);
      }
    }
  }
  // std::cout << "computing collision grid viewpoint cloud size: " << viewpoint_cloud->points.size() << std::endl;
  kdtree->setInputCloud(viewpoint_cloud);
  std::vector<int> nearby_viewpoint_indices;
  std::vector<float> nearby_viewpoint_sqdist;
  int count = 0;
  for (int x = 0; x < vp_.kCollisionGridSize.x(); x++)
  {
    for (int y = 0; y < vp_.kCollisionGridSize.y(); y++)
    {
      for (int z = 0; z < vp_.kCollisionGridSize.z(); z++)
      {
        Eigen::Vector3d query_point_position = collision_grid_->Sub2Pos(x, y, z);
        pcl::PointXYZI query_point;
        query_point.x = query_point_position.x();
        query_point.y = query_point_position.y();
        query_point.z = query_point_position.z();
        query_point.z *= vp_.kCollisionGridZScale;
        kdtree->radiusSearch(query_point, vp_.kCollisionRadius, nearby_viewpoint_indices, nearby_viewpoint_sqdist);
        int grid_ind = collision_grid_->Sub2Ind(x, y, z);
        for (int i = 0; i < nearby_viewpoint_indices.size(); i++)
        {
          int ind = nearby_viewpoint_indices[i];
          int viewpoint_ind = (int)(viewpoint_cloud->points[ind].intensity);
          MY_ASSERT(viewpoint_ind >= 0 && viewpoint_ind < vp_.kViewPointNum);
          collision_grid_->GetCell(grid_ind).push_back(viewpoint_ind);
        }
      }
    }
  }

  timer.Stop(false);
}

bool ViewPointManager::UpdateRobotPosition(const Eigen::Vector3d& robot_position)
{
  robot_position_ = robot_position;
  if (!initialized_)
  {
    initialized_ = true;
    UpdateOrigin();

    for (int x = 0; x < vp_.kNum.x(); x++)
    {
      for (int y = 0; y < vp_.kNum.y(); y++)
      {
        for (int z = 0; z < vp_.kNum.z(); z++)
        {
          int ind = grid_->Sub2Ind(Eigen::Vector3i(x, y, z));
          geometry_msgs::Point position;
          position.x = origin_.x() + x * vp_.kResol.x() + vp_.kResol.x() / 2.0;
          position.y = origin_.y() + y * vp_.kResol.y() + vp_.kResol.y() / 2.0;
          position.z = robot_position.z();
          SetViewPointPosition(ind, position, true);
          ResetViewPoint(ind, true);
        }
      }
    }
  }
  Eigen::Vector3i robot_grid_sub;
  Eigen::Vector3d diff = robot_position_ - origin_;
  Eigen::Vector3i sub = Eigen::Vector3i::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    robot_grid_sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / (vp_.kRolloverStepsize(i) * vp_.kResol(i))) : -1;
  }

  Eigen::Vector3i sub_diff = Eigen::Vector3i::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    sub_diff(i) = (vp_.kNum(i) / vp_.kRolloverStepsize(i)) / 2 - robot_grid_sub(i);
  }

  if (sub_diff.x() == 0 && sub_diff.y() == 0 && sub_diff.z() == 0)
  {
    return false;
  }

  Eigen::Vector3i rollover_step;
  rollover_step.x() = std::abs(sub_diff.x()) > 0 ?
                          vp_.kRolloverStepsize.x() * ((sub_diff.x() > 0) ? 1 : -1) * std::abs(sub_diff.x()) :
                          0;
  rollover_step.y() = std::abs(sub_diff.y()) > 0 ?
                          vp_.kRolloverStepsize.y() * ((sub_diff.y() > 0) ? 1 : -1) * std::abs(sub_diff.y()) :
                          0;
  rollover_step.z() = std::abs(sub_diff.z()) > 0 ?
                          vp_.kRolloverStepsize.z() * ((sub_diff.z() > 0) ? 1 : -1) * std::abs(sub_diff.z()) :
                          0;

  // std::cout << "rolling x: " << rollover_step.x() << " y: " << rollover_step.y() << " z: " << rollover_step.z()
  //           << std::endl;
  grid_->Roll(rollover_step);

  misc_utils_ns::Timer reset_timer("reset viewpoint");
  reset_timer.Start();

  //   origin_ = origin_ - rollover_step.cast<double>() * vp_.kResol;
  origin_.x() -= rollover_step.x() * vp_.kResol.x();
  origin_.y() -= rollover_step.y() * vp_.kResol.y();
  origin_.z() -= rollover_step.z() * vp_.kResol.z();

  grid_->GetUpdatedIndices(updated_viewpoint_indices_);
  for (const auto& ind : updated_viewpoint_indices_)
  {
    MY_ASSERT(grid_->InRange(ind));
    Eigen::Vector3i sub = grid_->Ind2Sub(ind);
    geometry_msgs::Point new_position;
    new_position.x = origin_.x() + sub.x() * vp_.kResol.x() + vp_.kResol.x() / 2.0;
    new_position.y = origin_.y() + sub.y() * vp_.kResol.y() + vp_.kResol.y() / 2.0;
    new_position.z = robot_position_.z();
    SetViewPointPosition(ind, new_position);
    ResetViewPoint(ind);
  }
  reset_timer.Stop(false);
  return true;
}

void ViewPointManager::UpdateOrigin()
{
  for (int i = 0; i < vp_.dimension_; i++)
  {
    origin_(i) = robot_position_(i) - (vp_.kResol(i) * vp_.kNum(i)) / 2.0;
  }
}

Eigen::Vector3i ViewPointManager::GetViewPointSub(Eigen::Vector3d position)
{
  Eigen::Vector3d diff = position - origin_;
  Eigen::Vector3i sub = Eigen::Vector3i::Zero();
  for (int i = 0; i < vp_.dimension_; i++)
  {
    sub(i) = diff(i) > 0 ? static_cast<int>(diff(i) / vp_.kResol(i)) : -1;
  }
  return sub;
}

int ViewPointManager::GetViewPointInd(Eigen::Vector3d position)
{
  Eigen::Vector3i sub = GetViewPointSub(position);
  if (grid_->InRange(sub))
  {
    return grid_->Sub2Ind(sub);
  }
  else
  {
    return -1;
  }
}

void ViewPointManager::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->clear();
  for (int i = 0; i < vp_.kViewPointNum; i++)
  {
    if (IsViewPointCandidate(i, true))
    {
      geometry_msgs::Point position = GetViewPointPosition(i, true);
      pcl::PointXYZI vis_point;
      vis_point.x = position.x;
      vis_point.y = position.y;
      vis_point.z = position.z;
      // vis_point.intensity = graph_index_map_[i];
      // if (viewpoints_[i].Visited())
      if (ViewPointVisited(i, true))
      {
        vis_point.intensity = -1.0;
      }
      else
      {
        vis_point.intensity = GetViewPointCoveredPointNum(i, true);
        vis_point.intensity += i * 1.0 / 10000.0;
      }
      // if (viewpoints_[i].InCurrentFrameLineOfSight())
      // {
      //   vis_point.intensity = 100;
      // }
      // else
      // {
      //   vis_point.intensity = -1;
      // }
      vis_cloud->points.push_back(vis_point);
    }
  }
}

void ViewPointManager::CheckViewPointCollisionWithCollisionGrid(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud)
{
  for (int i = 0; i < viewpoints_.size(); i++)
  {
    if (ViewPointInCollision(i, true))
    {
      AddViewPointCollisionFrameCount(i, true);
    }
  }
  std::fill(collision_point_count_.begin(), collision_point_count_.end(), 0);
  collision_grid_origin_ = origin_ - Eigen::Vector3d::Ones() * vp_.kCollisionRadius;
  collision_grid_->SetOrigin(collision_grid_origin_);
  for (const auto& point : collision_cloud->points)
  {
    Eigen::Vector3i collision_grid_sub = collision_grid_->Pos2Sub(point.x, point.y, point.z);
    if (collision_grid_->InRange(collision_grid_sub))
    {
      int collision_grid_ind = collision_grid_->Sub2Ind(collision_grid_sub);
      collision_point_count_[collision_grid_ind]++;
      if (collision_point_count_[collision_grid_ind] >= vp_.kCollisionPointThr)
      {
        std::vector<int> collision_viewpoint_indices = collision_grid_->GetCellValue(collision_grid_ind);
        for (int i = 0; i < collision_viewpoint_indices.size(); i++)
        {
          int viewpoint_ind = collision_viewpoint_indices[i];
          MY_ASSERT(viewpoint_ind >= 0 && viewpoint_ind < vp_.kViewPointNum);
          if (std::abs(point.z - GetViewPointHeight(viewpoint_ind)) <= vp_.kCollisionRadius)
          {
            SetViewPointCollision(viewpoint_ind, true);
          }
          if (point.intensity < 0.5)
          {
            ResetViewPointCollisionFrameCount(viewpoint_ind);
          }
        }
      }
    }
  }
}

bool ViewPointManager::InCollision(const Eigen::Vector3d& position)
{
  int viewpoint_ind = GetViewPointInd(position);
  bool node_in_collision = false;
  if (InRange(viewpoint_ind) &&
      std::abs(GetViewPointHeight(viewpoint_ind) - position.z()) < std::max(vp_.kResol.x(), vp_.kResol.y()) * 2)
  {
    if (ViewPointInCollision(viewpoint_ind))
    {
      return true;
    }
  }
  return false;
}

bool ViewPointManager::InCurrentFrameLineOfSight(const Eigen::Vector3d& position)
{
  int viewpoint_ind = GetViewPointInd(position);
  bool in_line_of_sight = false;
  if (InRange(viewpoint_ind))
  {
    if (ViewPointInCurrentFrameLineOfSight(viewpoint_ind))
    {
      return true;
    }
  }
  return false;
}

void ViewPointManager::CheckViewPointBoundaryCollision()
{
  // Check for the polygon boundary and nogo zones
  for (int i = 0; i < vp_.kViewPointNum; i++)
  {
    geometry_msgs::Point viewpoint_position = GetViewPointPosition(i, true);
    if ((!viewpoint_boundary_.points.empty() &&
         !misc_utils_ns::PointInPolygon(viewpoint_position, viewpoint_boundary_)))
    {
      SetViewPointCollision(i, true, true);
      continue;
    }
    for (int j = 0; j < nogo_boundary_.size(); j++)
    {
      if (!nogo_boundary_[j].points.empty() && misc_utils_ns::PointInPolygon(viewpoint_position, nogo_boundary_[j]))
      {
        SetViewPointCollision(i, true, true);

        break;
      }
    }
  }
}

void ViewPointManager::CheckViewPointCollision(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud)
{
  CheckViewPointCollisionWithCollisionGrid(collision_cloud);
  CheckViewPointBoundaryCollision();
}

void ViewPointManager::CheckViewPointCollisionWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                                          double collision_threshold)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr collision_cloud(new pcl::PointCloud<pcl::PointXYZI>());
  for (const auto& point : terrain_cloud->points)
  {
    if (point.intensity > collision_threshold)
    {
      collision_cloud->points.push_back(point);
    }
  }
  CheckViewPointCollisionWithCollisionGrid(collision_cloud);
}

void ViewPointManager::CheckViewPointLineOfSightHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                                       const Eigen::Vector3i& max_sub, const Eigen::Vector3i& min_sub)
{
  if (end_sub == start_sub)
    return;
  int viewpoint_ind = grid_->Sub2Ind(end_sub);
  geometry_msgs::Point viewpoint_position = GetViewPointPosition(viewpoint_ind);
  std::vector<Eigen::Vector3i> ray_cast_cells;
  misc_utils_ns::RayCast(start_sub, end_sub, max_sub, min_sub, ray_cast_cells);
  if (ray_cast_cells.size() > 1)
  {
    if (vp_.kLineOfSightStopAtNearestObstacle)
    {
      bool occlude = false;
      for (int i = 1; i < ray_cast_cells.size(); i++)
      {
        int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
        if (ViewPointInCollision(viewpoint_ind) && GetViewPointCollisionFrameCount(viewpoint_ind) == 0)
        {
          occlude = true;
          break;
        }
        if (!occlude)
        {
          SetViewPointInLineOfSight(viewpoint_ind, true);
          if (vp_.kCheckDynamicObstacleCollision &&
              GetViewPointCollisionFrameCount(viewpoint_ind) > vp_.kCollisionFrameCountMax)

          {
            SetViewPointCollision(viewpoint_ind, false);
          }
        }
      }
    }
    else
    {
      bool hit_obstacle = false;
      bool in_line_of_sight = false;
      for (int i = ray_cast_cells.size() - 1; i >= 0; i--)
      {
        int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
        if (ViewPointInCollision(viewpoint_ind) && GetViewPointCollisionFrameCount(viewpoint_ind) == 0)
        {
          hit_obstacle = true;
        }
        if (hit_obstacle && !ViewPointInCollision(viewpoint_ind))
        {
          in_line_of_sight = true;
        }
        if (hit_obstacle && ViewPointInCollision(viewpoint_ind) &&
            GetViewPointCollisionFrameCount(viewpoint_ind) > vp_.kCollisionFrameCountMax)

        {
          in_line_of_sight = true;
          if (vp_.kCheckDynamicObstacleCollision)
          {
            SetViewPointCollision(viewpoint_ind, false);
          }
        }
        if (in_line_of_sight)
        {
          SetViewPointInLineOfSight(viewpoint_ind, true);
        }
      }
      if (!hit_obstacle)
      {
        for (int i = ray_cast_cells.size() - 1; i >= 0; i--)
        {
          int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
          SetViewPointInLineOfSight(viewpoint_ind, true);
        }
      }
    }

    // Set in current frame line of sight
    bool occlude = false;
    for (int i = 1; i < ray_cast_cells.size(); i++)
    {
      int viewpoint_ind = grid_->Sub2Ind(ray_cast_cells[i]);
      if (ViewPointInCollision(viewpoint_ind) && GetViewPointCollisionFrameCount(viewpoint_ind) == 0)
      {
        occlude = true;
        break;
      }
      if (!occlude)
      {
        SetViewPointInCurrentFrameLineOfSight(viewpoint_ind, true);
      }
    }
  }
}

void ViewPointManager::CheckViewPointLineOfSight()
{
  if (!initialized_)
    return;

  for (int i = 0; i < viewpoints_.size(); i++)
  {
    SetViewPointInCurrentFrameLineOfSight(i, false, true);
  }

  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  MY_ASSERT(grid_->InRange(robot_sub));
  int robot_viewpoint_ind = grid_->Sub2Ind(robot_sub);
  SetViewPointInLineOfSight(robot_viewpoint_ind, true);
  SetViewPointInCurrentFrameLineOfSight(robot_viewpoint_ind, true);

  std::vector<bool> checked(vp_.kViewPointNum, false);
  std::vector<Eigen::Vector3i> ray_cast_cells;
  Eigen::Vector3i max_sub(vp_.kNum.x() - 1, vp_.kNum.y() - 1, vp_.kNum.z() - 1);
  Eigen::Vector3i min_sub(0, 0, 0);

  int x_indices[2] = { 0, vp_.kNum.x() - 1 };
  int y_indices[2] = { 0, vp_.kNum.y() - 1 };
  int z_indices[2] = { 0, vp_.kNum.z() - 1 };

  for (int xi = 0; xi < 2; xi++)
  {
    for (int y = 0; y < vp_.kNum.y(); y++)
    {
      for (int z = 0; z < vp_.kNum.z(); z++)
      {
        int x = x_indices[xi];
        Eigen::Vector3i end_sub(x, y, z);
        int array_ind = grid_->GetArrayInd(end_sub);
        if (!checked[array_ind])
        {
          CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
          checked[array_ind] = true;
        }
      }
    }
  }

  for (int x = 0; x < vp_.kNum.x(); x++)
  {
    for (int yi = 0; yi < 2; yi++)
    {
      for (int z = 0; z < vp_.kNum.z(); z++)
      {
        int y = y_indices[yi];
        Eigen::Vector3i end_sub(x, y, z);
        int array_ind = grid_->GetArrayInd(end_sub);
        if (!checked[array_ind])
        {
          CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
          checked[array_ind] = true;
        }
      }
    }
  }

  for (int x = 0; x < vp_.kNum.x(); x++)
  {
    for (int y = 0; y < vp_.kNum.y(); y++)
    {
      for (int zi = 0; zi < 2; zi++)
      {
        int z = z_indices[zi];
        Eigen::Vector3i end_sub(x, y, z);
        int array_ind = grid_->GetArrayInd(end_sub);
        if (!checked[array_ind])
        {
          CheckViewPointLineOfSightHelper(robot_sub, end_sub, max_sub, min_sub);
          checked[array_ind] = true;
        }
      }
    }
  }
}

void ViewPointManager::CheckViewPointInFOV()
{
  if (!initialized_)
    return;
  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  MY_ASSERT(grid_->InRange(robot_sub));
  for (int i = 0; i < vp_.kViewPointNum; i++)
  {
    geometry_msgs::Point viewpoint_position = GetViewPointPosition(i, true);
    if (!InRobotFOV(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z)))
    {
      SetViewPointInLineOfSight(i, false, true);
    }
  }
  int robot_viewpoint_ind = grid_->Sub2Ind(robot_sub);
  SetViewPointInLineOfSight(robot_viewpoint_ind, true);
}

bool ViewPointManager::InFOV(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position)
{
  Eigen::Vector3d diff = point_position - viewpoint_position;
  double xy_diff = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
  double z_diff = std::abs(diff.z());
  if (z_diff < vp_.kVerticalFOVRatio * xy_diff)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool ViewPointManager::InFOVAndRange(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position)
{
  Eigen::Vector3d diff = point_position - viewpoint_position;
  double z_diff = std::abs(diff.z());
  if (z_diff > vp_.kDiffZMax)
  {
    return false;
  }
  double xy_diff = sqrt(diff.x() * diff.x() + diff.y() * diff.y());
  if (xy_diff > vp_.kSensorRange)
  {
    return false;
  }
  if (z_diff < vp_.kVerticalFOVRatio * xy_diff)
  {
    return true;
  }
  else
  {
    return false;
  }
}

bool ViewPointManager::InRobotFOV(const Eigen::Vector3d& position)
{
  return InFOV(position, robot_position_);
}

void ViewPointManager::CheckViewPointConnectivity()
{
  if (!initialized_)
    return;
  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  MY_ASSERT(grid_->InRange(robot_sub));
  int robot_ind = grid_->Sub2Ind(robot_sub);
  int robot_array_ind = grid_->GetArrayInd(robot_sub);
  if (ViewPointInCollision(robot_ind))
  {
    // std::cout << "ViewPointManager::CheckViewPointConnectivity: robot in collision" << std::endl;
    // return;
    // Find the nearest viewpoint that is not in collision
    bool found_collision_free_viewpoint = false;
    double min_dist_to_robot = DBL_MAX;
    for (int i = 0; i < vp_.kViewPointNum; i++)
    {
      int array_ind = grid_->GetArrayInd(i);
      if (!ViewPointInCollision(i))
      {
        geometry_msgs::Point position = GetViewPointPosition(i);
        Eigen::Vector3d viewpoint_position(position.x, position.y, position.z);
        double dist_to_robot = (viewpoint_position - robot_position_).norm();
        if (dist_to_robot < min_dist_to_robot)
        {
          min_dist_to_robot = dist_to_robot;
          robot_ind = i;
          robot_array_ind = array_ind;
          found_collision_free_viewpoint = true;
        }
      }
    }
    if (!found_collision_free_viewpoint)
    {
      std::cout << "All viewpoints in collision, exisiting" << std::endl;
      return;
    }
  }
  for (auto& viewpoint : viewpoints_)
  {
    viewpoint.SetConnected(false);
  }
  std::vector<bool> checked(vp_.kViewPointNum, false);
  checked[robot_ind] = true;
  SetViewPointConnected(robot_ind, true);
  std::list<int> queue;
  queue.push_back(robot_ind);
  int connected_viewpoint_count = 1;
  while (!queue.empty())
  {
    int cur_ind = queue.front();
    queue.pop_front();
    for (int i = 0; i < connected_neighbor_indices_[cur_ind].size(); i++)
    {
      int neighbor_ind = connected_neighbor_indices_[cur_ind][i];
      if (!grid_->InRange(neighbor_ind))
      {
        std::cout << "ViewPointManager::CheckViewPointConnectivity: neighbor ind out of bound" << std::endl;
        continue;
      }
      if (!checked[neighbor_ind] && !ViewPointInCollision(neighbor_ind) && ViewPointInLineOfSight(neighbor_ind))
      {
        if (std::abs(GetViewPointHeight(cur_ind) - GetViewPointHeight(neighbor_ind)) < vp_.kCollisionCheckTerrainThr)
        {
          SetViewPointConnected(neighbor_ind, true);
          connected_viewpoint_count++;
          queue.push_back(neighbor_ind);
        }
      }
      checked[neighbor_ind] = true;
    }
  }
}

void ViewPointManager::UpdateViewPointVisited(const std::vector<Eigen::Vector3d>& positions)
{
  if (!initialized_)
    return;

  for (const auto& position : positions)
  {
    if (!InLocalPlanningHorizon(position))
    {
      continue;
    }
    Eigen::Vector3i viewpoint_sub = GetViewPointSub(position);
    if (grid_->InRange(viewpoint_sub))
    {
      int viewpoint_ind = grid_->Sub2Ind(viewpoint_sub);
      SetViewPointVisited(viewpoint_ind, true);
      for (const auto& neighbor_viewpoint_ind : in_range_neighbor_indices_[viewpoint_ind])
      {
        MY_ASSERT(grid_->InRange(neighbor_viewpoint_ind));
        SetViewPointVisited(neighbor_viewpoint_ind, true);
        int neighbor_array_ind = grid_->GetArrayInd(neighbor_viewpoint_ind);
      }
    }
  }
}

void ViewPointManager::UpdateViewPointVisited(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world)
{
  for (int i = 0; i < viewpoints_.size(); i++)
  {
    geometry_msgs::Point viewpoint_position = GetViewPointPosition(i, true);
    int cell_ind = grid_world->GetCellInd(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z);
    if (grid_world->IndInBound((cell_ind)))
    {
      grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(cell_ind);
      if (cell_status == grid_world_ns::CellStatus::COVERED_BY_OTHERS)
      {
        SetViewPointVisited(i, true, true);
      }
    }
  }
}

void ViewPointManager::SetBindingViewPointVisited(int array_ind)
{
  MY_ASSERT(InRange(array_ind));
  if (binding_viewpoint_array_indices_[array_ind].empty())
  {
    return;
  }
  for (const auto& binding_array_ind : binding_viewpoint_array_indices_[array_ind])
  {
    SetViewPointVisited(binding_array_ind, true, true);
    SetBindingViewPointVisited(binding_array_ind);
  }
}

void ViewPointManager::SetViewPointHeightWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                                     double terrain_height_threshold)
{
  // Set the height of the viewpoint nearby the robot to be the height of the robot, in case there is no terrain cloud
  // within the blind spot.
  if (!initialized_)
    return;
  Eigen::Vector3i robot_sub = GetViewPointSub(robot_position_);
  int robot_ind = grid_->Sub2Ind(robot_sub);
  MY_ASSERT(grid_->InRange(robot_sub));
  geometry_msgs::Point robot_viewpoint_position = GetViewPointPosition(robot_ind);

  if (!ViewPointHasTerrainHeight(robot_ind) ||
      std::abs(robot_viewpoint_position.z - robot_position_.z()) > vp_.kViewPointHeightFromTerrainChangeThreshold)
  {
    robot_viewpoint_position.z = robot_position_.z();
    SetViewPointPosition(robot_ind, robot_viewpoint_position);
    for (int i = 0; i < in_range_neighbor_indices_[robot_ind].size(); i++)
    {
      int neighbor_ind = in_range_neighbor_indices_[robot_ind][i];
      MY_ASSERT(grid_->InRange(neighbor_ind));
      if (!ViewPointHasTerrainHeight(neighbor_ind) ||
          std::abs(GetViewPointHeight(neighbor_ind) - robot_position_.z()) > 0.6)
      {
        SetViewPointHeight(neighbor_ind, robot_position_.z());
      }
    }
  }

  // Set the height of other viewpoints
  for (const auto& terrain_point : terrain_cloud->points)
  {
    if (terrain_point.intensity > terrain_height_threshold)
    {
      continue;
    }
    Eigen::Vector3i viewpoint_sub = GetViewPointSub(Eigen::Vector3d(terrain_point.x, terrain_point.y, terrain_point.z));
    if (grid_->InRange(viewpoint_sub))
    {
      int viewpoint_ind = grid_->Sub2Ind(viewpoint_sub);
      double target_height = terrain_point.z + vp_.kViewPointHeightFromTerrain;
      // If the viewpoint has not been set height with terrain points, or if there is a terrain point with a lower
      // height
      if (!ViewPointHasTerrainHeight(viewpoint_ind) || target_height < GetViewPointHeight(viewpoint_ind))
      {
        if (std::abs(target_height - GetViewPointHeight(viewpoint_ind)) >
            vp_.kViewPointHeightFromTerrainChangeThreshold)
        {
          ResetViewPoint(viewpoint_ind);
        }
        SetViewPointHeight(viewpoint_ind, target_height);
        SetViewPointHasTerrainHeight(viewpoint_ind, true);
      }
    }
  }

  // For viewpoints that are not set heights with terrain directly, use neighbors' heights
  for (int i = 0; i < vp_.kViewPointNum; i++)
  {
    if (!ViewPointHasTerrainHeight(i))
    {
      for (const auto& neighbor_ind : in_range_neighbor_indices_[i])
      {
        MY_ASSERT(grid_->InRange(neighbor_ind));
        if (ViewPointHasTerrainHeight(neighbor_ind))
        {
          double neighbor_height = GetViewPointHeight(neighbor_ind);
          if (std::abs(neighbor_height - GetViewPointHeight(i)) > vp_.kViewPointHeightFromTerrainChangeThreshold)
          {
            geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);
            viewpoint_position.z = neighbor_height;
            ResetViewPoint(i);
            SetViewPointPosition(i, viewpoint_position);
          }
          else
          {
            SetViewPointHeight(i, neighbor_height);
          }
        }
      }
    }
  }
}

// Reset viewpoint
void ViewPointManager::ResetViewPoint(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].Reset();
}
// Collision
bool ViewPointManager::ViewPointInCollision(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InCollision();
}
void ViewPointManager::SetViewPointCollision(int viewpoint_ind, bool in_collision, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetInCollision(in_collision);
}
// Line of Sight
bool ViewPointManager::ViewPointInLineOfSight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InLineOfSight();
}
void ViewPointManager::SetViewPointInLineOfSight(int viewpoint_ind, bool in_line_of_sight, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetInLineOfSight(in_line_of_sight);
}
// Connectivity
bool ViewPointManager::ViewPointConnected(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].Connected();
}
void ViewPointManager::SetViewPointConnected(int viewpoint_ind, bool connected, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetConnected(connected);
}
// Visited
bool ViewPointManager::ViewPointVisited(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].Visited();
}
void ViewPointManager::SetViewPointVisited(int viewpoint_ind, bool visited, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetVisited(visited);
}
// Selected
bool ViewPointManager::ViewPointSelected(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].Selected();
}
void ViewPointManager::SetViewPointSelected(int viewpoint_ind, bool selected, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetSelected(selected);
}
// Candidacy
bool ViewPointManager::IsViewPointCandidate(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].IsCandidate();
}
void ViewPointManager::SetViewPointCandidate(int viewpoint_ind, bool candidate, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetCandidate(candidate);
}
// Terrain Height
bool ViewPointManager::ViewPointHasTerrainHeight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].HasTerrainHeight();
}
void ViewPointManager::SetViewPointHasTerrainHeight(int viewpoint_ind, bool has_terrain_height, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetHasTerrainHeight(has_terrain_height);
}
// In exploring cell
bool ViewPointManager::ViewPointInExploringCell(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InExploringCell();
}
void ViewPointManager::SetViewPointInExploringCell(int viewpoint_ind, bool in_exploring_cell, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetInExploringCell(in_exploring_cell);
}
// Height
double ViewPointManager::GetViewPointHeight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetHeight();
}
void ViewPointManager::SetViewPointHeight(int viewpoint_ind, double height, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetHeight(height);
}
// In current frame line of sight
bool ViewPointManager::ViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].InCurrentFrameLineOfSight();
}
void ViewPointManager::SetViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool in_current_frame_line_of_sight,
                                                             bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetInCurrentFrameLineOfSight(in_current_frame_line_of_sight);
}
// Position
geometry_msgs::Point ViewPointManager::GetViewPointPosition(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetPosition();
}
void ViewPointManager::SetViewPointPosition(int viewpoint_ind, geometry_msgs::Point position, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetPosition(position);
}
// Cell Ind
int ViewPointManager::GetViewPointCellInd(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCellInd();
}
void ViewPointManager::SetViewPointCellInd(int viewpoint_ind, int cell_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].SetCellInd(cell_ind);
}
// Collision frame count
int ViewPointManager::GetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCollisionFrameCount();
}
void ViewPointManager::AddViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].AddCollisionFrame();
}
void ViewPointManager::ResetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].ResetCollisionFrameCount();
}
// Covered point list
void ViewPointManager::ResetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].ResetCoveredPointList();
  viewpoints_[array_ind].ResetCoveredFrontierPointList();
}
void ViewPointManager::AddUncoveredPoint(int viewpoint_ind, int point_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].AddCoveredPoint(point_ind);
}
void ViewPointManager::AddUncoveredFrontierPoint(int viewpoint_ind, int point_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  viewpoints_[array_ind].AddCoveredFrontierPoint(point_ind);
}
const std::vector<int>& ViewPointManager::GetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind) const
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredPointList();
}
const std::vector<int>& ViewPointManager::GetViewPointCoveredFrontierPointList(int viewpoint_ind,
                                                                               bool use_array_ind) const
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredFrontierPointList();
}

int ViewPointManager::GetViewPointCoveredPointNum(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredPointNum();
}

int ViewPointManager::GetViewPointCoveredFrontierPointNum(int viewpoint_ind, bool use_array_ind)
{
  int array_ind = GetViewPointArrayInd(viewpoint_ind, use_array_ind);
  return viewpoints_[array_ind].GetCoveredFrontierPointNum();
}

int ViewPointManager::GetViewPointCoveredPointNum(const std::vector<bool>& point_list, int viewpoint_index,
                                                  bool use_array_ind)
{
  int covered_point_num = 0;
  for (const auto& point_ind : GetViewPointCoveredPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
    if (!point_list[point_ind])
    {
      covered_point_num++;
    }
  }
  return covered_point_num;
}
int ViewPointManager::GetViewPointCoveredFrontierPointNum(const std::vector<bool>& frontier_point_list,
                                                          int viewpoint_index, bool use_array_ind)
{
  int covered_frontier_point_num = 0;
  for (const auto& point_ind : GetViewPointCoveredFrontierPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
    if (!frontier_point_list[point_ind])
    {
      covered_frontier_point_num++;
    }
  }
  return covered_frontier_point_num;
}

void ViewPointManager::UpdateViewPointCoveredPoint(std::vector<bool>& point_list, int viewpoint_index,
                                                   bool use_array_ind)
{
  for (const auto& point_ind : GetViewPointCoveredPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
    point_list[point_ind] = true;
  }
}
void ViewPointManager::UpdateViewPointCoveredFrontierPoint(std::vector<bool>& frontier_point_list, int viewpoint_index,
                                                           bool use_array_ind)
{
  for (const auto& point_ind : GetViewPointCoveredFrontierPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
    frontier_point_list[point_ind] = true;
  }
}

int ViewPointManager::GetViewPointCandidate()
{
  viewpoint_candidate_cloud_->clear();
  viewpoint_in_collision_cloud_->clear();
  candidate_indices_.clear();
  for (int i = 0; i < vp_.kViewPointNum; i++)
  {
    SetViewPointCandidate(i, false);
    if (!ViewPointInCollision(i) && ViewPointInLineOfSight(i) && ViewPointConnected(i))
    {
      SetViewPointCandidate(i, true);
      candidate_indices_.push_back(i);
      geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);
      pcl::PointXYZI point;
      point.x = viewpoint_position.x;
      point.y = viewpoint_position.y;
      point.z = viewpoint_position.z;
      viewpoint_candidate_cloud_->points.push_back(point);
    }
    if (ViewPointInCollision(i))
    {
      geometry_msgs::Point viewpoint_position = GetViewPointPosition(i);
      pcl::PointXYZI point;
      point.x = viewpoint_position.x;
      point.y = viewpoint_position.y;
      point.z = viewpoint_position.z;
      point.intensity = GetViewPointCollisionFrameCount(i);
      viewpoint_in_collision_cloud_->points.push_back(point);
    }
  }
  // std::cout << "candidate viewpoint num: " << candidate_indices_.size() << std::endl;
  if (!candidate_indices_.empty())
  {
    kdtree_viewpoint_candidate_->setInputCloud(viewpoint_candidate_cloud_);
  }

  if (!viewpoint_in_collision_cloud_->points.empty())
  {
    kdtree_viewpoint_in_collision_->setInputCloud(viewpoint_in_collision_cloud_);
  }

  // Construct a graph of all the viewpoints
  GetCandidateViewPointGraph(candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_);

  return candidate_indices_.size();
}

nav_msgs::Path ViewPointManager::GetViewPointShortestPath(const Eigen::Vector3d& start_position,
                                                          const Eigen::Vector3d& target_position)
{
  nav_msgs::Path path;
  if (!InLocalPlanningHorizon(start_position))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start position " << start_position.transpose()
                                                                                 << " not in local planning horizon");
    return path;
  }
  if (!InLocalPlanningHorizon(target_position))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
                                                                                  << " not in local planning horizon");
    return path;
  }
  int start_viewpoint_ind = GetNearestCandidateViewPointInd(start_position);
  int target_viewpoint_ind = GetNearestCandidateViewPointInd(target_position);

  int start_graph_ind = graph_index_map_[start_viewpoint_ind];
  int target_graph_ind = graph_index_map_[target_viewpoint_ind];

  std::vector<int> path_graph_indices;
  double path_length =
      misc_utils_ns::AStarSearch(candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_,
                                 start_graph_ind, target_graph_ind, true, path_graph_indices);
  if (path_graph_indices.size() >= 2)
  {
    for (int i = 0; i < path_graph_indices.size(); i++)
    {
      int graph_idx = path_graph_indices[i];
      int ind = candidate_indices_[graph_idx];
      geometry_msgs::PoseStamped pose;
      pose.pose.position = GetViewPointPosition(ind);
      path.poses.push_back(pose);
    }
  }
  return path;
}

bool ViewPointManager::GetViewPointShortestPathWithMaxLength(const Eigen::Vector3d& start_position,
                                                             const Eigen::Vector3d& target_position,
                                                             double max_path_length, nav_msgs::Path& path)
{
  if (!InLocalPlanningHorizon(start_position))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start position " << start_position.transpose()
                                                                                 << " not in local planning horizon");
    return false;
  }
  if (!InLocalPlanningHorizon(target_position))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target position " << target_position.transpose()
                                                                                  << " not in local planning horizon");
    return false;
  }
  int start_viewpoint_ind = GetNearestCandidateViewPointInd(start_position);
  int target_viewpoint_ind = GetNearestCandidateViewPointInd(target_position);
  int start_graph_ind = graph_index_map_[start_viewpoint_ind];
  int target_graph_ind = graph_index_map_[target_viewpoint_ind];

  std::vector<int> path_graph_indices;
  double shortest_path_dist = 0;
  bool found_path = misc_utils_ns::AStarSearchWithMaxPathLength(
      candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_, start_graph_ind,
      target_graph_ind, true, path_graph_indices, shortest_path_dist, max_path_length);

  if (found_path && path_graph_indices.size() >= 2)
  {
    for (int i = 0; i < path_graph_indices.size(); i++)
    {
      int graph_idx = path_graph_indices[i];
      int ind = candidate_indices_[graph_idx];
      geometry_msgs::PoseStamped pose;
      pose.pose.position = GetViewPointPosition(ind);
      path.poses.push_back(pose);
    }
  }
  return found_path;
}

void ViewPointManager::UpdateCandidateViewPointCellStatus(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world)
{
  for (const auto& ind : candidate_indices_)
  {
    int cell_ind = GetViewPointCellInd(ind);
    if (grid_world->IndInBound(cell_ind))
    {
      grid_world_ns::CellStatus cell_status = grid_world->GetCellStatus(cell_ind);
      if (cell_status == grid_world_ns::CellStatus::UNSEEN || cell_status == grid_world_ns::CellStatus::EXPLORING)
      {
        SetViewPointInExploringCell(ind, true);
      }
      else
      {
        SetViewPointInExploringCell(ind, false);
      }
    }
    else
    {
      ROS_WARN_STREAM("ViewPointManager::UpdateCandidateViewPointCellStatus: cell ind " << cell_ind << " out of bound");
    }
  }
}

void ViewPointManager::GetCandidateViewPointGraph(std::vector<std::vector<int>>& graph,
                                                  std::vector<std::vector<double>>& dist,
                                                  std::vector<geometry_msgs::Point>& positions)
{
  graph.clear();
  dist.clear();
  positions.clear();
  if (candidate_indices_.empty())
  {
    return;
  }
  graph.resize(candidate_indices_.size());
  dist.resize(graph.size());

  for (int i = 0; i < candidate_indices_.size(); i++)
  {
    int ind = candidate_indices_[i];
    graph_index_map_[ind] = i;
  }

  // Build the graph
  for (int i = 0; i < candidate_indices_.size(); i++)
  {
    int cur_ind = candidate_indices_[i];
    positions.push_back(GetViewPointPosition(cur_ind));
    for (int j = 0; j < connected_neighbor_indices_[cur_ind].size(); j++)
    {
      int neighbor_ind = connected_neighbor_indices_[cur_ind][j];
      double neighbor_dist = connected_neighbor_dist_[cur_ind][j];
      if (IsViewPointCandidate(neighbor_ind))
      {
        graph[i].push_back(graph_index_map_[neighbor_ind]);
        dist[i].push_back(neighbor_dist);
      }
    }
  }
}

void ViewPointManager::SelectViewPoint(const std::vector<std::pair<int, int>>& queue, const std::vector<bool>& covered,
                                       std::vector<int>& selected_viewpoint_indices, bool use_frontier)
{
  if (use_frontier)
  {
    if (queue.empty() || queue[0].first < vp_.kMinAddFrontierPointNum)
    {
      return;
    }
  }
  else
  {
    if (queue.empty() || queue[0].first < vp_.kMinAddPointNum)
    {
      return;
    }
  }

  std::vector<bool> covered_copy;
  for (int i = 0; i < covered.size(); i++)
  {
    covered_copy.push_back(covered[i]);
  }
  std::vector<std::pair<int, int>> queue_copy;
  for (int i = 0; i < queue.size(); i++)
  {
    queue_copy.push_back(queue[i]);
  }

  int sample_range = 0;
  for (int i = 0; i < queue_copy.size(); i++)
  {
    if (use_frontier)
    {
      if (queue_copy[i].first >= vp_.kMinAddFrontierPointNum)
      {
        sample_range++;
      }
    }
    else
    {
      if (queue_copy[i].first >= vp_.kMinAddPointNum)
      {
        sample_range++;
      }
    }
  }

  sample_range = std::min(vp_.kGreedyViewPointSampleRange, sample_range);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);
  int queue_idx = gen_next_queue_idx(gen);
  int cur_ind = queue_copy[queue_idx].second;

  while (true)
  {
    int cur_array_ind = grid_->GetArrayInd(cur_ind);
    if (use_frontier)
    {
      for (const auto& point_ind : GetViewPointCoveredFrontierPointList(cur_array_ind, true))

      {
        MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
        if (!covered_copy[point_ind])
        {
          covered_copy[point_ind] = true;
        }
      }
    }
    else
    {
      for (const auto& point_ind : GetViewPointCoveredPointList(cur_array_ind, true))
      {
        MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
        if (!covered_copy[point_ind])
        {
          covered_copy[point_ind] = true;
        }
      }
    }
    selected_viewpoint_indices.push_back(cur_ind);
    queue_copy.erase(queue_copy.begin() + queue_idx);

    // Update the queue
    for (int i = 0; i < queue_copy.size(); i++)
    {
      int add_point_num = 0;
      int ind = queue_copy[i].second;
      int array_ind = grid_->GetArrayInd(ind);
      if (use_frontier)
      {
        for (const auto& point_ind : GetViewPointCoveredFrontierPointList(array_ind, true))
        {
          MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
          if (!covered_copy[point_ind])
          {
            add_point_num++;
          }
        }
      }
      else
      {
        for (const auto& point_ind : GetViewPointCoveredPointList(array_ind, true))
        {
          MY_ASSERT(misc_utils_ns::InRange<bool>(covered_copy, point_ind));
          if (!covered_copy[point_ind])
          {
            add_point_num++;
          }
        }
      }

      queue_copy[i].first = add_point_num;
    }

    std::sort(queue_copy.begin(), queue_copy.end(), SortPairInRev);

    if (queue_copy.empty() || queue_copy[0].first < vp_.kMinAddPointNum)
    {
      break;
    }
    if (use_frontier)
    {
      if (queue_copy.empty() || queue_copy[0].first < vp_.kMinAddFrontierPointNum)
      {
        break;
      }
    }

    // Randomly select the next point
    int sample_range = 0;
    for (int i = 0; i < queue.size(); i++)
    {
      if (use_frontier)
      {
        if (queue[i].first >= vp_.kMinAddFrontierPointNum)
        {
          sample_range++;
        }
      }
      else
      {
        if (queue[i].first >= vp_.kMinAddPointNum)
        {
          sample_range++;
        }
      }
    }
    sample_range = std::min(vp_.kGreedyViewPointSampleRange, sample_range);
    std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);
    queue_idx = gen_next_queue_idx(gen);
    cur_ind = queue_copy[queue_idx].second;
  }
}

void ViewPointManager::SelectViewPointFromFrontierQueue(std::vector<std::pair<int, int>>& frontier_queue,
                                                        std::vector<bool>& frontier_covered,
                                                        std::vector<int>& selected_viewpoint_indices)
{
  if (vp_.kUseFrontier && !frontier_queue.empty() && frontier_queue[0].first > vp_.kMinAddFrontierPointNum)
  {
    // Update the frontier queue
    for (const auto& ind : selected_viewpoint_indices)
    {
      UpdateViewPointCoveredFrontierPoint(frontier_covered, ind);
    }
    for (int i = 0; i < frontier_queue.size(); i++)
    {
      int ind = frontier_queue[i].second;
      int covered_frontier_point_num = GetViewPointCoveredFrontierPointNum(frontier_covered, ind);
      frontier_queue[i].first = covered_frontier_point_num;
    }
    std::sort(frontier_queue.begin(), frontier_queue.end(), SortPairInRev);
    SelectViewPoint(frontier_queue, frontier_covered, selected_viewpoint_indices, true);
  }
}

exploration_path_ns::ExplorationPath ViewPointManager::SolveTSP(const std::vector<int>& selected_viewpoint_indices,
                                                                std::vector<int>& ordered_viewpoint_indices)

{
  // nav_msgs::Path tsp_path;
  exploration_path_ns::ExplorationPath tsp_path;

  if (selected_viewpoint_indices.empty())
  {
    return tsp_path;
  }

  // Get start and end index
  int start_ind = selected_viewpoint_indices.size() - 1;
  int end_ind = selected_viewpoint_indices.size() - 1;
  int robot_ind = 0;
  int lookahead_ind = 0;

  for (int i = 0; i < selected_viewpoint_indices.size(); i++)
  {
    if (selected_viewpoint_indices[i] == start_viewpoint_ind_)
    {
      start_ind = i;
    }
    if (selected_viewpoint_indices[i] == end_viewpoint_ind_)
    {
      end_ind = i;
    }
    if (selected_viewpoint_indices[i] == robot_viewpoint_ind_)
    {
      robot_ind = i;
    }
    if (selected_viewpoint_indices[i] == lookahead_viewpoint_ind_)
    {
      lookahead_ind = i;
    }
  }

  bool has_start_end_dummy = start_ind != end_ind;
  bool has_robot_lookahead_dummy = robot_ind != lookahead_ind;

  // Get distance matrix
  int node_size;
  if (has_start_end_dummy && has_robot_lookahead_dummy)
  {
    node_size = selected_viewpoint_indices.size() + 2;
  }
  else if (has_start_end_dummy || has_robot_lookahead_dummy)
  {
    node_size = selected_viewpoint_indices.size() + 1;
  }
  else
  {
    node_size = selected_viewpoint_indices.size();
  }
  misc_utils_ns::Timer find_path_timer("find path");
  find_path_timer.Start();
  std::vector<std::vector<int>> distance_matrix(node_size, std::vector<int>(node_size, 0));
  std::vector<int> tmp;
  for (int i = 0; i < selected_viewpoint_indices.size(); i++)
  {
    int from_ind = selected_viewpoint_indices[i];
    int from_graph_idx = graph_index_map_[from_ind];
    for (int j = 0; j < i; j++)
    {
      int to_ind = selected_viewpoint_indices[j];
      int to_graph_idx = graph_index_map_[to_ind];
      double path_length =
          misc_utils_ns::AStarSearch(candidate_viewpoint_graph_, candidate_viewpoint_dist_,
                                     candidate_viewpoint_position_, from_graph_idx, to_graph_idx, false, tmp);
      distance_matrix[i][j] = static_cast<int>(10 * path_length);
    }
  }

  for (int i = 0; i < selected_viewpoint_indices.size(); i++)
  {
    for (int j = i + 1; j < selected_viewpoint_indices.size(); j++)
    {
      distance_matrix[i][j] = distance_matrix[j][i];
    }
  }

  // Add a dummy node to connect the start and end nodes
  if (has_start_end_dummy && has_robot_lookahead_dummy)
  {
    int start_end_dummy_node_ind = node_size - 1;
    int robot_lookahead_dummy_node_ind = node_size - 2;
    for (int i = 0; i < selected_viewpoint_indices.size(); i++)
    {
      if (i == start_ind || i == end_ind)
      {
        distance_matrix[i][start_end_dummy_node_ind] = 0;
        distance_matrix[start_end_dummy_node_ind][i] = 0;
      }
      else
      {
        distance_matrix[i][start_end_dummy_node_ind] = 9999;
        distance_matrix[start_end_dummy_node_ind][i] = 9999;
      }
      if (i == robot_ind || i == lookahead_ind)
      {
        distance_matrix[i][robot_lookahead_dummy_node_ind] = 0;
        distance_matrix[robot_lookahead_dummy_node_ind][i] = 0;
      }
      else
      {
        distance_matrix[i][robot_lookahead_dummy_node_ind] = 9999;
        distance_matrix[robot_lookahead_dummy_node_ind][i] = 9999;
      }
    }

    distance_matrix[start_end_dummy_node_ind][robot_lookahead_dummy_node_ind] = 9999;
    distance_matrix[robot_lookahead_dummy_node_ind][start_end_dummy_node_ind] = 9999;
  }
  else if (has_start_end_dummy)
  {
    int end_node_ind = node_size - 1;
    for (int i = 0; i < selected_viewpoint_indices.size(); i++)
    {
      if (i == start_ind || i == end_ind)
      {
        distance_matrix[i][end_node_ind] = 0;
        distance_matrix[end_node_ind][i] = 0;
      }
      else
      {
        distance_matrix[i][end_node_ind] = 9999;
        distance_matrix[end_node_ind][i] = 9999;
      }
    }
  }
  else if (has_robot_lookahead_dummy)
  {
    int end_node_ind = node_size - 1;
    for (int i = 0; i < selected_viewpoint_indices.size(); i++)
    {
      if (i == robot_ind || i == lookahead_ind)
      {
        distance_matrix[i][end_node_ind] = 0;
        distance_matrix[end_node_ind][i] = 0;
      }
      else
      {
        distance_matrix[i][end_node_ind] = 9999;
        distance_matrix[end_node_ind][i] = 9999;
      }
    }
  }

  find_path_timer.Stop(false);
  find_path_runtime_ += find_path_timer.GetDuration(kRuntimeUnit);

  misc_utils_ns::Timer tsp_timer("tsp");
  tsp_timer.Start();

  tsp_solver_ns::DataModel data;
  data.distance_matrix = distance_matrix;
  data.depot = start_ind;

  tsp_solver_ns::TSPSolver tsp_solver(data);
  tsp_solver.Solve();

  std::vector<int> path_index;
  if (has_start_end_dummy)
  {
    tsp_solver.getSolutionNodeIndex(path_index, true);
  }
  else
  {
    tsp_solver.getSolutionNodeIndex(path_index, false);
  }

  // Get rid of the dummy node connecting the robot and lookahead point
  for (int i = 0; i < path_index.size(); i++)
  {
    if (path_index[i] >= selected_viewpoint_indices.size() || path_index[i] < 0)
    {
      path_index.erase(path_index.begin() + i);
      i--;
    }
  }

  ordered_viewpoint_indices.clear();
  for (int i = 0; i < path_index.size(); i++)
  {
    ordered_viewpoint_indices.push_back(selected_viewpoint_indices[path_index[i]]);
  }

  // Add the end node index
  if (start_ind == end_ind && !path_index.empty())
  {
    path_index.push_back(path_index[0]);
  }

  tsp_timer.Stop(false);
  tsp_runtime_ += tsp_timer.GetDuration(kRuntimeUnit);

  if (path_index.size() > 1)
  {
    int cur_ind;
    int next_ind;
    int from_graph_idx;
    int to_graph_idx;

    for (int i = 0; i < path_index.size() - 1; i++)
    {
      cur_ind = selected_viewpoint_indices[path_index[i]];
      next_ind = selected_viewpoint_indices[path_index[i + 1]];

      from_graph_idx = graph_index_map_[cur_ind];
      to_graph_idx = graph_index_map_[next_ind];

      // Add viewpoint node
      // int cur_array_ind = grid_->GetArrayInd(cur_ind);
      // geometry_msgs::Point cur_node_position = viewpoints_[cur_array_ind].GetPosition();
      geometry_msgs::Point cur_node_position = GetViewPointPosition(cur_ind);
      exploration_path_ns::Node cur_node(cur_node_position, exploration_path_ns::NodeType::LOCAL_VIEWPOINT);
      cur_node.local_viewpoint_ind_ = cur_ind;
      if (cur_ind == robot_viewpoint_ind_)
      {
        cur_node.type_ = exploration_path_ns::NodeType::ROBOT;
      }
      else if (cur_ind == lookahead_viewpoint_ind_)
      {
        int covered_point_num = GetViewPointCoveredPointNum(cur_ind);
        int covered_frontier_num = GetViewPointCoveredFrontierPointNum(cur_ind);
        if (covered_point_num > vp_.kMinAddPointNum || covered_frontier_num > vp_.kMinAddFrontierPointNum)
        {
          cur_node.type_ = exploration_path_ns::NodeType::LOCAL_VIEWPOINT;
        }
        else
        {
          cur_node.type_ = exploration_path_ns::NodeType::LOOKAHEAD_POINT;
        }
      }
      else if (cur_ind == start_viewpoint_ind_)
      {
        cur_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
      }
      else if (cur_ind == end_viewpoint_ind_)
      {
        cur_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
      }
      tsp_path.Append(cur_node);

      std::vector<int> path_graph_indices;
      misc_utils_ns::AStarSearch(candidate_viewpoint_graph_, candidate_viewpoint_dist_, candidate_viewpoint_position_,
                                 from_graph_idx, to_graph_idx, true, path_graph_indices);
      // Add viapoint nodes;
      if (path_graph_indices.size() > 2)
      {
        for (int j = 1; j < path_graph_indices.size() - 1; j++)
        {
          int graph_idx = path_graph_indices[j];
          int ind = candidate_indices_[graph_idx];
          exploration_path_ns::Node node;
          node.type_ = exploration_path_ns::NodeType::LOCAL_VIA_POINT;
          node.local_viewpoint_ind_ = ind;
          geometry_msgs::Point node_position = GetViewPointPosition(ind);
          node.position_.x() = node_position.x;
          node.position_.y() = node_position.y;
          node.position_.z() = node_position.z;
          tsp_path.Append(node);
        }
      }

      geometry_msgs::Point next_node_position = GetViewPointPosition(next_ind);
      exploration_path_ns::Node next_node(next_node_position, exploration_path_ns::NodeType::LOCAL_VIEWPOINT);
      next_node.local_viewpoint_ind_ = next_ind;
      if (next_ind == robot_viewpoint_ind_)
      {
        next_node.type_ = exploration_path_ns::NodeType::ROBOT;
      }
      else if (next_ind == lookahead_viewpoint_ind_)
      {
        next_node.type_ = exploration_path_ns::NodeType::LOOKAHEAD_POINT;
      }
      else if (next_ind == start_viewpoint_ind_)
      {
        next_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_START;
      }
      else if (next_ind == end_viewpoint_ind_)
      {
        next_node.type_ = exploration_path_ns::NodeType::LOCAL_PATH_END;
      }
      tsp_path.Append(next_node);
    }
  }

  return tsp_path;
}

int ViewPointManager::GetNearestCandidateViewPointInd(const Eigen::Vector3d& position)
{
  int viewpoint_ind = GetViewPointInd(position);
  if (InRange(viewpoint_ind))
  {
    if (IsViewPointCandidate(viewpoint_ind))
    {
      return viewpoint_ind;
    }
  }
  if (!candidate_indices_.empty())
  {
    // Find the closest viewpoint that is a candidate viewpoint
    double min_dist = DBL_MAX;
    int min_dist_ind = -1;
    geometry_msgs::Point query_position;
    query_position.x = position.x();
    query_position.y = position.y();
    query_position.z = position.z();
    for (const auto& cur_viewpoint_ind : candidate_indices_)
    {
      geometry_msgs::Point cur_position = GetViewPointPosition(cur_viewpoint_ind);
      double dist =
          misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(cur_position, query_position);
      if (dist < min_dist)
      {
        min_dist = dist;
        min_dist_ind = cur_viewpoint_ind;
      }
    }
    return min_dist_ind;
  }
  else
  {
    std::cout << "Candidate viewpoint empty, can't find nearest candidate viewpoints to the position" << std::endl;
    return -1;
  }
}

int ViewPointManager::GetBoundaryViewpointIndex(const exploration_path_ns::ExplorationPath& global_path)
{
  int boundary_viewpoint_index = robot_viewpoint_ind_;
  if (!global_path.nodes_.empty())
  {
    if (InLocalPlanningHorizon(global_path.nodes_.front().position_))
    {
      for (int i = 0; i < global_path.nodes_.size(); i++)
      {
        if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
            global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
            !InLocalPlanningHorizon(global_path.nodes_[i].position_))
        {
          break;
        }
        boundary_viewpoint_index = GetNearestCandidateViewPointInd(global_path.nodes_[i].position_);
      }
    }
  }
  return boundary_viewpoint_index;
}

void ViewPointManager::GetBoundaryViewpointIndices(exploration_path_ns::ExplorationPath global_path)
{
  start_viewpoint_ind_ = GetBoundaryViewpointIndex(global_path);
  global_path.Reverse();
  end_viewpoint_ind_ = GetBoundaryViewpointIndex(global_path);
}

void ViewPointManager::GetNavigationViewPointIndices(exploration_path_ns::ExplorationPath global_path,
                                                     std::vector<int>& navigation_viewpoint_indices)
{
  // Get start and end point
  robot_viewpoint_ind_ = GetNearestCandidateViewPointInd(robot_position_);
  lookahead_viewpoint_ind_ = GetNearestCandidateViewPointInd(lookahead_position_);
  if (!lookahead_point_update_ || !grid_->InRange(lookahead_viewpoint_ind_))
  {
    lookahead_viewpoint_ind_ = robot_viewpoint_ind_;
  }
  // Get connecting viewpoints to the global path
  GetBoundaryViewpointIndices(global_path);

  // Update the coverage with viewpoints that must visit
  navigation_viewpoint_indices.push_back(start_viewpoint_ind_);
  navigation_viewpoint_indices.push_back(end_viewpoint_ind_);
  navigation_viewpoint_indices.push_back(robot_viewpoint_ind_);
  navigation_viewpoint_indices.push_back(lookahead_viewpoint_ind_);
}

void ViewPointManager::EnqueueViewpointCandidates(std::vector<std::pair<int, int>>& cover_point_queue,
                                                  std::vector<std::pair<int, int>>& frontier_queue,
                                                  const std::vector<bool>& covered_point_list,
                                                  const std::vector<bool>& covered_frontier_point_list,
                                                  const std::vector<int>& selected_viewpoint_array_indices)
{
  for (const auto& viewpoint_index : candidate_indices_)
  {
    if (ViewPointVisited(viewpoint_index) || !ViewPointInExploringCell(viewpoint_index))
    {
      continue;
    }
    int viewpoint_array_index = grid_->GetArrayInd(viewpoint_index);
    if (std::find(selected_viewpoint_array_indices.begin(), selected_viewpoint_array_indices.end(),
                  viewpoint_array_index) != selected_viewpoint_array_indices.end())
    {
      continue;
    }
    int covered_point_num = GetViewPointCoveredPointNum(covered_point_list, viewpoint_array_index, true);
    if (covered_point_num >= vp_.kMinAddPointNum)
    {
      cover_point_queue.emplace_back(covered_point_num, viewpoint_index);
    }
    else if (vp_.kUseFrontier)
    {
      int covered_frontier_point_num =
          GetViewPointCoveredFrontierPointNum(covered_frontier_point_list, viewpoint_array_index, true);
      if (covered_frontier_point_num >= vp_.kMinAddFrontierPointNum)
      {
        frontier_queue.emplace_back(covered_frontier_point_num, viewpoint_index);
      }
    }
  }

  // Sort the queue
  std::sort(cover_point_queue.begin(), cover_point_queue.end(), SortPairInRev);
  if (vp_.kUseFrontier)
  {
    std::sort(frontier_queue.begin(), frontier_queue.end(), SortPairInRev);
  }
}

bool ViewPointManager::MoveViewPointInFOV(int& viewpoint_array_ind)
{
  MY_ASSERT(grid_->InRange(viewpoint_array_ind));
  if (!ViewPointInLineOfSight(viewpoint_array_ind, true) && !ViewPointInCollision(viewpoint_array_ind, true))
  {
    int ind = grid_->GetInd(viewpoint_array_ind);
    Eigen::Vector3i sub = grid_->Ind2Sub(ind);
    // Move up the viewpoint until hits a candidate viewpoint
    int z = sub.z() + 1;
    while (z < vp_.kNum.z())
    {
      int cur_ind = grid_->Sub2Ind(Eigen::Vector3i(sub.x(), sub.y(), z));
      int cur_array_ind = grid_->GetArrayInd(cur_ind);
      if (IsViewPointCandidate(cur_array_ind, true))
      {
        binding_viewpoint_array_indices_[cur_array_ind].push_back(viewpoint_array_ind);
        bound_viewpoint_array_ind_[viewpoint_array_ind] = cur_array_ind;
        viewpoint_array_ind = cur_array_ind;
        return true;
        // std::cout << "viewpoint " << array_ind << " bound to " << cur_array_ind << std::endl;
      }
      z++;
    }
  }
  return false;
}

exploration_path_ns::ExplorationPath ViewPointManager::SolveLocalTSP(
    const exploration_path_ns::ExplorationPath& global_path, int uncovered_point_num, int uncovered_frontier_point_num)
{
  exploration_path_ns::ExplorationPath local_path;

  find_path_runtime_ = 0;
  viewpoint_sampling_runtime_ = 0;
  tsp_runtime_ = 0;

  local_coverage_complete_ = false;

  misc_utils_ns::Timer find_path_timer("find path");
  find_path_timer.Start();

  std::vector<int> navigation_viewpoint_indices;
  GetNavigationViewPointIndices(global_path, navigation_viewpoint_indices);

  find_path_timer.Stop(false);
  find_path_runtime_ += find_path_timer.GetDuration(kRuntimeUnit);

  // Sampling viewpoints
  misc_utils_ns::Timer viewpoint_sampling_timer("viewpoint sampling");
  viewpoint_sampling_timer.Start();

  std::vector<bool> covered(uncovered_point_num, false);
  std::vector<bool> frontier_covered(uncovered_frontier_point_num, false);

  std::vector<int> pre_selected_viewpoint_array_indices;
  std::vector<int> reused_viewpoint_indices;
  for (auto& viewpoint_array_ind : last_selected_viewpoint_array_indices_)
  {
    if (ViewPointVisited(viewpoint_array_ind, true) || !IsViewPointCandidate(viewpoint_array_ind, true))
    {
      continue;
    }
    int covered_point_num = GetViewPointCoveredPointNum(covered, viewpoint_array_ind, true);
    if (covered_point_num >= vp_.kMinAddPointNum)
    {
      reused_viewpoint_indices.push_back(grid_->GetInd(viewpoint_array_ind));
    }
    else if (vp_.kUseFrontier)
    {
      int covered_frontier_point_num = GetViewPointCoveredFrontierPointNum(frontier_covered, viewpoint_array_ind, true);
      if (covered_frontier_point_num >= vp_.kMinAddFrontierPointNum)
      {
        reused_viewpoint_indices.push_back(grid_->GetInd(viewpoint_array_ind));
      }
    }
  }

  for (const auto& ind : reused_viewpoint_indices)
  {
    int viewpoint_array_ind = grid_->GetArrayInd(ind);
    pre_selected_viewpoint_array_indices.push_back(viewpoint_array_ind);
  }
  for (const auto& ind : navigation_viewpoint_indices)
  {
    int array_ind = grid_->GetArrayInd(ind);
    pre_selected_viewpoint_array_indices.push_back(array_ind);
  }

  // Update coverage
  for (auto& viewpoint_array_ind : pre_selected_viewpoint_array_indices)
  {
    // Update covered points and frontiers
    UpdateViewPointCoveredPoint(covered, viewpoint_array_ind, true);
    if (vp_.kUseFrontier)
    {
      UpdateViewPointCoveredFrontierPoint(frontier_covered, viewpoint_array_ind, true);
    }
  }

  // Enqueue candidate viewpoints
  std::vector<std::pair<int, int>> queue;
  std::vector<std::pair<int, int>> frontier_queue;
  EnqueueViewpointCandidates(queue, frontier_queue, covered, frontier_covered, pre_selected_viewpoint_array_indices);

  viewpoint_sampling_timer.Stop(false, kRuntimeUnit);
  viewpoint_sampling_runtime_ += viewpoint_sampling_timer.GetDuration(kRuntimeUnit);

  std::vector<int> ordered_viewpoint_indices;
  if (!queue.empty() && queue[0].first > vp_.kMinAddPointNum)
  {
    double min_path_length = DBL_MAX;
    for (int itr = 0; itr < vp_.kLocalPathOptimizationItrMax; itr++)
    {
      std::vector<int> selected_viewpoint_indices_itr;

      // Select from the queue
      misc_utils_ns::Timer select_viewpoint_timer("select viewpoints");
      select_viewpoint_timer.Start();
      SelectViewPoint(queue, covered, selected_viewpoint_indices_itr, false);
      SelectViewPointFromFrontierQueue(frontier_queue, frontier_covered, selected_viewpoint_indices_itr);

      // Add viewpoints from last planning cycle
      for (const auto& ind : reused_viewpoint_indices)
      {
        selected_viewpoint_indices_itr.push_back(ind);
      }
      // Add viewpoints for navigation
      for (const auto& ind : navigation_viewpoint_indices)
      {
        selected_viewpoint_indices_itr.push_back(ind);
      }

      misc_utils_ns::UniquifyIntVector(selected_viewpoint_indices_itr);

      select_viewpoint_timer.Stop(false, kRuntimeUnit);
      viewpoint_sampling_runtime_ += select_viewpoint_timer.GetDuration(kRuntimeUnit);

      // Solve the TSP problem
      exploration_path_ns::ExplorationPath local_path_itr;
      local_path_itr = SolveTSP(selected_viewpoint_indices_itr, ordered_viewpoint_indices);

      double path_length = local_path_itr.GetLength();
      if (!local_path_itr.nodes_.empty() && path_length < min_path_length)

      {
        min_path_length = path_length;
        local_path = local_path_itr;
        last_selected_viewpoint_indices_ = ordered_viewpoint_indices;
      }
    }
  }
  else
  {
    misc_utils_ns::Timer select_viewpoint_timer("viewpoint sampoing");
    select_viewpoint_timer.Start();

    // std::cout << "entering tsp routine" << std::endl;
    std::vector<int> selected_viewpoint_indices_itr;

    // Add viewpoints from last planning cycle
    for (const auto& ind : reused_viewpoint_indices)
    {
      selected_viewpoint_indices_itr.push_back(ind);
    }
    SelectViewPointFromFrontierQueue(frontier_queue, frontier_covered, selected_viewpoint_indices_itr);

    if (selected_viewpoint_indices_itr.empty())
    {
      local_coverage_complete_ = true;
    }

    // Add viewpoints for navigation
    for (const auto& ind : navigation_viewpoint_indices)
    {
      selected_viewpoint_indices_itr.push_back(ind);
    }

    misc_utils_ns::UniquifyIntVector(selected_viewpoint_indices_itr);

    select_viewpoint_timer.Stop(false, kRuntimeUnit);
    viewpoint_sampling_runtime_ += select_viewpoint_timer.GetDuration(kRuntimeUnit);

    local_path = SolveTSP(selected_viewpoint_indices_itr, ordered_viewpoint_indices);

    last_selected_viewpoint_indices_ = ordered_viewpoint_indices;
  }

  last_selected_viewpoint_array_indices_.clear();
  for (const auto& ind : last_selected_viewpoint_indices_)
  {
    int array_ind = grid_->GetArrayInd(ind);
    last_selected_viewpoint_array_indices_.push_back(array_ind);
  }

  for (int i = 0; i < viewpoints_.size(); i++)
  {
    SetViewPointSelected(i, false, true);
  }
  for (const auto& viewpoint_index : last_selected_viewpoint_indices_)
  {
    if (viewpoint_index != robot_viewpoint_ind_ && viewpoint_index != start_viewpoint_ind_ &&
        viewpoint_index != end_viewpoint_ind_ && viewpoint_index != lookahead_viewpoint_ind_)
    {
      SetViewPointSelected(viewpoint_index, true);
    }
  }
  return local_path;
}

void ViewPointManager::GetSelectedViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  cloud->clear();
  for (const auto& viewpoint_index : last_selected_viewpoint_indices_)
  {
    geometry_msgs::Point position = GetViewPointPosition(viewpoint_index);
    pcl::PointXYZI point;
    point.x = position.x;
    point.y = position.y;
    point.z = position.z;
    if (viewpoint_index == robot_viewpoint_ind_)
    {
      point.intensity = 0.0;
    }
    else if (viewpoint_index == start_viewpoint_ind_)
    {
      point.intensity = 1.0;
    }
    else if (viewpoint_index == end_viewpoint_ind_)
    {
      point.intensity = 2.0;
    }
    else
    {
      point.intensity = 3.0;
    }
    cloud->points.push_back(point);
  }
}

void ViewPointManager::GetCollisionViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  cloud->clear();
  for (const auto& point : viewpoint_in_collision_cloud_->points)
  {
    cloud->points.push_back(point);
  }
}

void ViewPointManager::GetViewPointHeightBounds(int viewpoint_ind, double& upper, double& lower)
{
  MY_ASSERT(grid_->InRange(viewpoint_ind));
  geometry_msgs::Point position = GetViewPointPosition(viewpoint_ind);
  upper = position.z;
  lower = position.z;
  Eigen::Vector3i viewpoint_sub = grid_->Ind2Sub(viewpoint_ind);
  if (viewpoint_sub.z() < vp_.kNum.z() - 1)
  {
    for (int z = viewpoint_sub.z() + 1; z < vp_.kNum.z(); z++)
    {
      Eigen::Vector3i sub = viewpoint_sub;
      sub.z() = z;
      int ind = grid_->Sub2Ind(sub);
      MY_ASSERT(grid_->InRange(ind));
      if (IsViewPointCandidate(ind))
      {
        upper += vp_.kResol.z();
      }
      else
      {
        break;
      }
    }
  }
  if (viewpoint_sub.z() > 0)
  {
    for (int z = viewpoint_sub.z() - 1; z >= 0; z--)
    {
      Eigen::Vector3i sub = viewpoint_sub;
      sub.z() = z;
      int ind = grid_->Sub2Ind(sub);
      MY_ASSERT(grid_->InRange(ind));
      if (IsViewPointCandidate(ind))
      {
        lower -= vp_.kResol.z();
      }
      else
      {
        break;
      }
    }
  }
}

void ViewPointManager::SetLookAheadPoint(const Eigen::Vector3d& lookahead_point)
{
  lookahead_position_ = lookahead_point;
  lookahead_point_update_ = true;
}

void ViewPointManager::GetViewPointCandidatePositionsAndValidity(std::vector<Eigen::Vector3d>& positions,
                                                                 std::vector<bool>& validity)
{
  positions.clear();
  validity.clear();

  Eigen::Vector3d position;
  geometry_msgs::Point point;
  for (int i = 0; i < vp_.kViewPointNum; i++)
  {
    point = GetViewPointPosition(i);
    position.x() = point.x;
    position.y() = point.y;
    position.z() = point.z;
    positions.push_back(position);
    if (IsViewPointCandidate(i))
    {
      validity.push_back(true);
    }
    else
    {
      validity.push_back(false);
    }
  }
}

int ViewPointManager::GetInRangeClosestViewPointInd(const geometry_msgs::Point& position, double range)
{
  std::vector<int> nearby_indices;
  std::vector<float> nearby_sqdist;
  pcl::PointXYZI point;
  point.x = position.x;
  point.y = position.y;
  point.z = position.z;
  double search_range = std::max(vp_.kResol.x(), vp_.kResol.y());
  search_range = std::max(search_range, vp_.kResol.z());
  // Add margins
  search_range += 0.2;
  kdtree_viewpoint_candidate_->radiusSearch(point, search_range, nearby_indices, nearby_sqdist);
  if (nearby_indices.size() >= 1)
  {
    float min_dist = DBL_MAX;
    int min_ind = 0;
    for (int i = 0; i < nearby_sqdist.size(); i++)
    {
      if (nearby_sqdist[i] < min_dist)
      {
        min_dist = nearby_sqdist[i];
        min_ind = i;
      }
    }
    MY_ASSERT(misc_utils_ns::InRange<int>(candidate_indices_, nearby_indices[min_ind]));
    return candidate_indices_[nearby_indices[min_ind]];
  }
  else
  {
    return -1;
  }
}

bool ViewPointManager::InLocalPlanningHorizon(const Eigen::Vector3d& position)
{
  int viewpoint_ind = GetViewPointInd(position);
  if (InRange(viewpoint_ind))
  {
    double max_z_diff = std::max(vp_.kResol.x(), vp_.kResol.y()) * 2;
    geometry_msgs::Point viewpoint_position = GetViewPointPosition(viewpoint_ind);
    if (std::abs(viewpoint_position.z - position.z()) < max_z_diff &&
        (IsViewPointCandidate(viewpoint_ind) || ViewPointInCollision(viewpoint_ind)))
    {
      return true;
    }
  }
  return false;
}

}  // namespace viewpoint_manager_ns