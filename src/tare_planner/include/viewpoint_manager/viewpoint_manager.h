/**
 * @file viewpoint_manager.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that manages the viewpoints inside the local planning horizon
 * @version 0.1
 * @date 2020-06-10
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <cmath>

#include <Eigen/Core>
// ROS
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point.h>
// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/PointIndices.h>

#include <grid/grid.h>
#include <rollable_grid/rollable_grid.h>
#include <viewpoint/viewpoint.h>
#include <utils/misc_utils.h>
#include <grid_world/grid_world.h>
#include <exploration_path/exploration_path.h>

namespace viewpoint_manager_ns
{
struct ViewPointManagerParameter
{
  // Layout
  bool kUseFrontier;
  int dimension_;
  int kViewPointNum;
  Eigen::Vector3i kNum;
  Eigen::Vector3i kRolloverStepsize;
  Eigen::Vector3d kResol;
  Eigen::Vector3d kBoundaryMax;
  Eigen::Vector3d kBoundaryMin;
  Eigen::Vector3d LocalPlanningHorizonSize;

  // Collision check
  double kCollisionCheckTerrainThr;
  double kViewPointZMax;
  double kViewPointZMin;

  // Collision grid
  Eigen::Vector3i kCollisionGridSize;
  Eigen::Vector3d kCollisionGridResolution;
  double kCollisionRadius;
  double kCollisionGridZScale;
  int kCollisionPointThr;

  // Line of Sight Check
  bool kLineOfSightStopAtNearestObstacle;
  bool kCheckDynamicObstacleCollision;
  int kCollisionFrameCountMax;

  // Terrain height
  double kViewPointHeightFromTerrain;
  double kViewPointHeightFromTerrainChangeThreshold;

  // Coverage
  double kCoverageOcclusionThr;
  double kCoverageDilationRadius;
  double kCoveragePointCloudResolution;

  // Distances
  double kSensorRange;
  double kVisitRange;
  double kNeighborRange;
  double kHeightFromTerrain;
  double kDistanceToIntConst;

  double kVerticalFOVRatio;
  double kDiffZMax;
  double kInFovXYDistThreshold;
  double kInFovZDiffThreshold;

  bool ReadParameters(ros::NodeHandle& nh);
};

class ViewPointManager
{
public:
  std::vector<int> candidate_indices_;
  explicit ViewPointManager(ros::NodeHandle& nh);
  ~ViewPointManager() = default;

  int GetViewPointArrayInd(int viewpoint_ind, bool use_array_ind = false) const;
  int GetViewPointInd(int viewpoint_array_ind) const;

  inline bool InRange(int viewpoint_ind)
  {
    return grid_->InRange(viewpoint_ind);
  }
  inline bool InRange(const Eigen::Vector3i& sub)
  {
    return grid_->InRange(sub);
  }
  inline int GetViewPointNum()
  {
    return viewpoints_.size();
  }
  Eigen::Vector3d GetResolution()
  {
    return vp_.kResol;
  }
  inline void UpdateViewPointBoundary(const geometry_msgs::Polygon& polygon)
  {
    viewpoint_boundary_ = polygon;
  }

  inline void UpdateNogoBoundary(const std::vector<geometry_msgs::Polygon>& nogo_boundary)
  {
    nogo_boundary_ = nogo_boundary;
  }
  bool UpdateRobotPosition(const Eigen::Vector3d& robot_position);
  void UpdateOrigin();
  Eigen::Vector3i GetViewPointSub(Eigen::Vector3d position);
  int GetViewPointInd(Eigen::Vector3d position);
  Eigen::Vector3d GetOrigin()
  {
    return origin_;
  }

  bool InCollision(const Eigen::Vector3d& position);
  bool InCurrentFrameLineOfSight(const Eigen::Vector3d& position);
  void CheckViewPointBoundaryCollision();
  void CheckViewPointCollisionWithCollisionGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud);
  void CheckViewPointCollision(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud);
  void CheckViewPointCollisionWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                          double collision_threshold);
  void CheckViewPointLineOfSightHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                       const Eigen::Vector3i& max_sub, const Eigen::Vector3i& min_sub);
  void CheckViewPointLineOfSight();
  void CheckViewPointInFOV();
  bool InFOV(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position);
  bool InFOVAndRange(const Eigen::Vector3d& point_position, const Eigen::Vector3d& viewpoint_position);
  bool InRobotFOV(const Eigen::Vector3d& position);
  void CheckViewPointConnectivity();
  void UpdateViewPointVisited(const std::vector<Eigen::Vector3d>& positions);
  void UpdateViewPointVisited(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world);
  void SetViewPointHeightWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                     double terrain_height_threshold = DBL_MAX);

  template <class PCLPointType>
  void UpdateViewPointCoverage(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    // std::cout << "update cloud size: " << cloud->points.size() << std::endl;
    int update_viewpoint_count = 0;
    for (auto& viewpoint : viewpoints_)
    {
      if (viewpoint.InCollision())
      {
        continue;
      }
      update_viewpoint_count++;
    }
    // std::cout << "update viewpoint num: " << update_viewpoint_count << std::endl;
    for (const auto& point : cloud->points)
    {
      for (int i = 0; i < viewpoints_.size(); i++)
      // for (auto& viewpoint : viewpoints_)
      {
        if (viewpoints_[i].InCollision())
        {
          continue;
        }
        geometry_msgs::Point viewpoint_position = viewpoints_[i].GetPosition();
        if (misc_utils_ns::InFOVSimple(
                Eigen::Vector3d(point.x, point.y, point.z),
                Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold, vp_.kInFovZDiffThreshold))
        {
          viewpoints_[i].UpdateCoverage<PCLPointType>(point);
        }
      }
    }
  }

  template <class PCLPointType>
  void UpdateRolledOverViewPointCoverage(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    for (const auto& point : cloud->points)
    {
      for (const auto& viewpoint_ind : updated_viewpoint_indices_)
      {
        int array_ind = grid_->GetArrayInd(viewpoint_ind);
        if (viewpoints_[array_ind].InCollision())
        {
          continue;
        }
        geometry_msgs::Point viewpoint_position = viewpoints_[array_ind].GetPosition();
        if (misc_utils_ns::InFOVSimple(
                Eigen::Vector3d(point.x, point.y, point.z),
                Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold, vp_.kInFovZDiffThreshold))
        {
          viewpoints_[array_ind].UpdateCoverage<PCLPointType>(point);
        }
      }
    }
  }

  inline double GetSensorRange() const
  {
    return vp_.kSensorRange;
  }
  inline double GetCoverageOcclusionThr() const
  {
    return vp_.kCoverageOcclusionThr;
  }
  inline double GetCoverageDilationRadius() const
  {
    return vp_.kCoverageDilationRadius;
  }

  template <class PointType>
  bool VisibleByViewPoint(const PointType& point, int viewpoint_ind)
  {
    MY_ASSERT(grid_->InRange(viewpoint_ind));
    int array_ind = grid_->GetArrayInd(viewpoint_ind);
    geometry_msgs::Point viewpoint_position = viewpoints_[array_ind].GetPosition();
    if (std::abs(point.z - viewpoint_position.z) > vp_.kDiffZMax)
    {
      return false;
    }
    if (!misc_utils_ns::InFOVSimple(Eigen::Vector3d(point.x, point.y, point.z),
                                    Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z),
                                    vp_.kVerticalFOVRatio, vp_.kSensorRange, vp_.kInFovXYDistThreshold,
                                    vp_.kInFovZDiffThreshold))
    {
      return false;
    }
    bool visible = viewpoints_[array_ind].CheckVisibility<PointType>(point, vp_.kCoverageOcclusionThr);

    return visible;
  }

  // Viewpoint management
  void ResetViewPoint(int viewpoint_ind, bool use_array_ind = false);

  bool ViewPointInCollision(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointCollision(int viewpoint_ind, bool in_collision, bool use_array_ind = false);

  bool ViewPointInLineOfSight(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointInLineOfSight(int viewpoint_ind, bool in_line_of_sight, bool use_array_ind = false);

  bool ViewPointConnected(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointConnected(int viewpoint_ind, bool connected, bool use_array_ind = false);

  bool ViewPointVisited(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointVisited(int viewpoint_ind, bool visited, bool use_array_ind = false);

  bool ViewPointSelected(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointSelected(int viewpoint_ind, bool selected, bool use_array_ind = false);

  bool IsViewPointCandidate(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointCandidate(int viewpoint_ind, bool candidate, bool use_array_ind = false);

  bool ViewPointHasTerrainHeight(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointHasTerrainHeight(int viewpoint_ind, bool has_terrain_height, bool use_array_ind = false);

  bool ViewPointInExploringCell(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointInExploringCell(int viewpoint_ind, bool in_exploring_cell, bool use_array_ind = false);

  double GetViewPointHeight(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointHeight(int viewpoint_ind, double height, bool use_array_ind = false);

  bool ViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointInCurrentFrameLineOfSight(int viewpoint_ind, bool in_current_frame_line_of_sight,
                                             bool use_array_ind = false);

  geometry_msgs::Point GetViewPointPosition(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointPosition(int viewpoint_ind, geometry_msgs::Point position, bool use_array_ind = false);

  int GetViewPointCellInd(int viewpoint_ind, bool use_array_ind = false);
  void SetViewPointCellInd(int viewpoint_ind, int cell_ind, bool use_array_ind = false);

  int GetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind = false);
  void AddViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind = false);
  void ResetViewPointCollisionFrameCount(int viewpoint_ind, bool use_array_ind = false);

  void ResetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind = false);
  void AddUncoveredPoint(int viewpoint_ind, int point_ind, bool use_array_ind = false);
  void AddUncoveredFrontierPoint(int viewpoint_ind, int point_ind, bool use_array_ind = false);
  const std::vector<int>& GetViewPointCoveredPointList(int viewpoint_ind, bool use_array_ind = false) const;
  const std::vector<int>& GetViewPointCoveredFrontierPointList(int viewpoint_ind, bool use_array_ind = false) const;

  int GetViewPointCoveredPointNum(int viewpoint_ind, bool use_array_ind = false);
  int GetViewPointCoveredFrontierPointNum(int viewpoint_ind, bool use_array_ind = false);
  int GetViewPointCoveredPointNum(const std::vector<bool>& point_list, int viewpoint_index, bool use_array_ind = false);
  int GetViewPointCoveredFrontierPointNum(const std::vector<bool>& frontier_point_list, int viewpoint_index,
                                          bool use_array_ind = false);
  void UpdateViewPointCoveredPoint(std::vector<bool>& point_list, int viewpoint_index, bool use_array_ind = false);
  void UpdateViewPointCoveredFrontierPoint(std::vector<bool>& frontier_point_list, int viewpoint_index,
                                           bool use_array_ind = false);

  int GetViewPointCandidate();
  std::vector<int> GetViewPointCandidateIndices() const
  {
    return candidate_indices_;
  }
  nav_msgs::Path GetViewPointShortestPath(int start_viewpoint_ind, int target_viewpoint_ind);
  nav_msgs::Path GetViewPointShortestPath(const Eigen::Vector3d& start_position,
                                          const Eigen::Vector3d& target_position);
  bool GetViewPointShortestPathWithMaxLength(const Eigen::Vector3d& start_position,
                                             const Eigen::Vector3d& target_position, double max_path_length,
                                             nav_msgs::Path& path);

  void UpdateCandidateViewPointCellStatus(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world);

  int GetNearestCandidateViewPointInd(const Eigen::Vector3d& position);
  bool InLocalPlanningHorizon(const Eigen::Vector3d& position);
  Eigen::Vector3d GetLocalPlanningHorizonSize()
  {
    return vp_.LocalPlanningHorizonSize;
  }
  bool UseFrontier()
  {
    return vp_.kUseFrontier;
  }
  // For visualization
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud);
  void GetCollisionViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

  typedef std::unique_ptr<ViewPointManager> Ptr;

private:
  void ComputeConnectedNeighborIndices();
  void ComputeInRangeNeighborIndices();
  void GetCandidateViewPointGraph(std::vector<std::vector<int>>& graph, std::vector<std::vector<double>>& dist,
                                  std::vector<geometry_msgs::Point>& positions);
  void GetCollisionCorrespondence();

  bool initialized_;
  ViewPointManagerParameter vp_;
  std::unique_ptr<rollable_grid_ns::RollableGrid> grid_;
  std::vector<viewpoint_ns::ViewPoint> viewpoints_;
  std::vector<std::vector<int>> connected_neighbor_indices_;
  std::vector<std::vector<double>> connected_neighbor_dist_;
  std::vector<std::vector<int>> in_range_neighbor_indices_;
  std::vector<int> updated_viewpoint_indices_;
  std::vector<int> graph_index_map_;
  Eigen::Vector3d robot_position_;
  Eigen::Vector3d origin_;
  Eigen::Vector3d collision_grid_origin_;
  Eigen::Vector3d local_planning_horizon_size_;
  std::unique_ptr<grid_ns::Grid<std::vector<int>>> collision_grid_;
  std::vector<int> collision_point_count_;
  std::vector<std::vector<int>> candidate_viewpoint_graph_;
  std::vector<std::vector<double>> candidate_viewpoint_dist_;
  std::vector<geometry_msgs::Point> candidate_viewpoint_position_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_viewpoint_candidate_;
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr kdtree_viewpoint_in_collision_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr viewpoint_candidate_cloud_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr viewpoint_in_collision_cloud_;

  geometry_msgs::Polygon viewpoint_boundary_;
  std::vector<geometry_msgs::Polygon> nogo_boundary_;
};

}  // namespace viewpoint_manager_ns