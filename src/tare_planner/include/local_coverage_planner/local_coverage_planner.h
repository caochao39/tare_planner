/**
 * @file local_coverage_planner.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that ensures coverage in the surroundings of the robot
 * @version 0.1
 * @date 2021-05-30
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <Eigen/Core>

#include <ros/ros.h>
#include <nav_msgs/Path.h>

#include <grid_world/grid_world.h>
#include <exploration_path/exploration_path.h>
#include <viewpoint_manager/viewpoint_manager.h>

namespace local_coverage_planner_ns
{
class LocalCoveragePlanner
{
public:
  explicit LocalCoveragePlanner(ros::NodeHandle& nh);
  ~LocalCoveragePlanner() = default;

  // Update representation
  bool UpdateRobotPosition(const Eigen::Vector3d& robot_position);
  void UpdateOrigin();
  Eigen::Vector3d GetOrigin()
  {
    return origin_;
  }
  inline void UpdateViewPointBoundary(const geometry_msgs::Polygon& polygon)
  {
    // viewpoint_boundary_ = polygon;
  }

  inline void UpdateNogoBoundary(const std::vector<geometry_msgs::Polygon>& nogo_boundary)
  {
    // nogo_boundary_ = nogo_boundary;
  }

  template <class PCLPointType>
  void UpdateViewPointCoverage(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    viewpoint_manager_->UpdateViewPointCoverage<PCLPointType>(cloud);
  }
  template <class PCLPointType>
  void UpdateRolledOverViewPointCoverage(const typename pcl::PointCloud<PCLPointType>::Ptr& cloud)
  {
    viewpoint_manager_->UpdateRolledOverViewPointCoverage<PCLPointType>(cloud);
  }

  template <class PointType>
  bool VisibleByViewPoint(const PointType& point, int viewpoint_ind)
  {
    return viewpoint_manager_->VisibleByViewPoint<PointType>(point, viewpoint_ind);
  }

  // Collision
  void CheckViewPointBoundaryCollision();
  void CheckViewPointCollisionWithCollisionGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud);
  void CheckViewPointCollision(const pcl::PointCloud<pcl::PointXYZI>::Ptr& collision_cloud);
  void SetViewPointHeightWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                     double terrain_height_threshold = DBL_MAX);
  void CheckViewPointCollisionWithTerrain(const pcl::PointCloud<pcl::PointXYZI>::Ptr& terrain_cloud,
                                          double collision_threshold);
  // Line of sight
  void CheckViewPointLineOfSightHelper(const Eigen::Vector3i& start_sub, const Eigen::Vector3i& end_sub,
                                       const Eigen::Vector3i& max_sub, const Eigen::Vector3i& min_sub);
  void CheckViewPointLineOfSight();

  // Connectivity
  void CheckViewPointInFOV();
  void CheckViewPointConnectivity();

  // Visited
  void UpdateViewPointVisited(const std::vector<Eigen::Vector3d>& positions);
  void UpdateViewPointVisited(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world);

  int GetViewPointCandidate();

  // Path planning
  nav_msgs::Path GetViewPointShortestPath(const Eigen::Vector3d& start_position,
                                          const Eigen::Vector3d& target_position);
  bool GetViewPointShortestPathWithMaxLength(const Eigen::Vector3d& start_position,
                                             const Eigen::Vector3d& target_position, double max_path_length,
                                             nav_msgs::Path& path);

  // Local coverage
  void SetLookAheadPoint(const Eigen::Vector3d& lookahead_point);
  void UpdateCandidateViewPointCellStatus(std::unique_ptr<grid_world_ns::GridWorld> const& grid_world);
  exploration_path_ns::ExplorationPath
  SolveLocalCoverageProblem(const exploration_path_ns::ExplorationPath& global_path, int uncovered_point_num,
                            int uncovered_frontier_point_num = 0);

  // Runtime
  int GetFindPathRuntime()
  {
    return find_path_runtime_;
  }
  int GetViewPointSamplingRuntime()
  {
    return viewpoint_sampling_runtime_;
  }
  int GetTSPRuntime()
  {
    return tsp_runtime_;
  }

  bool InLocalPlanningHorizon(const Eigen::Vector3d& position);
  bool InCollision(const Eigen::Vector3d& position);
  bool InCurrentFrameLineOfSight(const Eigen::Vector3d& position);
  bool IsLocalCoverageComplete()
  {
    return local_coverage_complete_;
  }

  //   Eigen::Vector3d GetLocalPlanningHorizonSize()
  //   {
  //     return vp_.LocalPlanningHorizonSize;
  //   }
  // Visualization
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud);
  void GetSelectedViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);
  void GetCollisionViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

private:
  void ComputeConnectedNeighborIndices();
  void ComputeInRangeNeighborIndices();
  void GetCollisionCorrespondence();
  void GetCandidateViewPointGraph(std::vector<std::vector<int>>& graph, std::vector<std::vector<double>>& dist,
                                  std::vector<geometry_msgs::Point>& positions);

  int GetNearestCandidateViewPointInd(const Eigen::Vector3d& position);

  void SelectViewPoint(const std::vector<std::pair<int, int>>& queue, const std::vector<bool>& covered,
                       std::vector<int>& selected_viewpoint_indices, bool use_frontier = false);
  void SelectViewPointFromFrontierQueue(std::vector<std::pair<int, int>>& frontier_queue,
                                        std::vector<bool>& frontier_covered,
                                        std::vector<int>& selected_viewpoint_indices);
  exploration_path_ns::ExplorationPath SolveTSP(const std::vector<int>& selected_viewpoint_indices,
                                                std::vector<int>& ordered_viewpoint_indices);

  viewpoint_manager_ns::ViewPointManager::Ptr viewpoint_manager_;

  Eigen::Vector3d origin_;

  // Runtime
  int find_path_runtime_;
  int viewpoint_sampling_runtime_;
  int tsp_runtime_;
  static const std::string kRuntimeUnit;

  bool local_coverage_complete_;
};
}  // namespace local_coverage_planner_ns