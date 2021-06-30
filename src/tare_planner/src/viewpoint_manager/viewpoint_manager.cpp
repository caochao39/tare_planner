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
  kSensorRange = misc_utils_ns::getParam<double>(nh, "kSensorRange", 10.0);
  kNeighborRange = misc_utils_ns::getParam<double>(nh, "kNeighborRange", 3.0);

  kVerticalFOVRatio = tan(M_PI / 15);
  kDiffZMax = kSensorRange * kVerticalFOVRatio;
  kInFovXYDistThreshold = 3 * (kCoveragePointCloudResolution / 2) / tan(M_PI / 15);
  kInFovZDiffThreshold = 3 * kCoveragePointCloudResolution;

  return true;
}

ViewPointManager::ViewPointManager(ros::NodeHandle& nh) : initialized_(false)
{
  vp_.ReadParameters(nh);

  kdtree_viewpoint_candidate_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  kdtree_viewpoint_in_collision_ = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
  viewpoint_candidate_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  viewpoint_in_collision_cloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  grid_ = std::make_unique<rolling_grid_ns::RollingGrid>(vp_.kNum);
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

int ViewPointManager::GetViewPointArrayInd(int viewpoint_ind, bool use_array_ind) const
{
  MY_ASSERT(grid_->InRange(viewpoint_ind));
  return (use_array_ind ? viewpoint_ind : grid_->GetArrayInd(viewpoint_ind));
}

int ViewPointManager::GetViewPointInd(int viewpoint_array_ind) const
{
  return grid_->GetInd(viewpoint_array_ind);
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

void ViewPointManager::ResetViewPointCoverage()
{
  for (auto& viewpoint : viewpoints_)
  {
    viewpoint.ResetCoverage();
  }
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

nav_msgs::Path ViewPointManager::GetViewPointShortestPath(int start_viewpoint_ind, int target_viewpoint_ind)
{
  nav_msgs::Path path;
  if (!InRange(start_viewpoint_ind))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath start viewpoint ind: " << start_viewpoint_ind
                                                                                       << " not in range");
    return path;
  }
  if (!InRange(target_viewpoint_ind))
  {
    ROS_WARN_STREAM("ViewPointManager::GetViewPointShortestPath target viewpoint ind: " << target_viewpoint_ind
                                                                                        << " not in range");
    return path;
  }

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

  return GetViewPointShortestPath(start_viewpoint_ind, target_viewpoint_ind);
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

void ViewPointManager::GetCollisionViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  cloud->clear();
  for (const auto& point : viewpoint_in_collision_cloud_->points)
  {
    cloud->points.push_back(point);
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