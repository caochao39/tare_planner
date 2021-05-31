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

#include "grid_world/grid_world.h"
#include "exploration_path/exploration_path.h"
#include "viewpoint_manager/viewpoint_manager.h"

namespace local_coverage_planner_ns
{
struct LocalCoveragePlannerParameter
{
  int kMinAddPointNum;
  int kMinAddFrontierPointNum;
  int kGreedyViewPointSampleRange;
  int kLocalPathOptimizationItrMax;

  bool ReadParameters(ros::NodeHandle& nh);
};
class LocalCoveragePlanner
{
public:
  explicit LocalCoveragePlanner(ros::NodeHandle& nh);
  ~LocalCoveragePlanner() = default;

  // Update representation
  void SetRobotPosition(const Eigen::Vector3d& robot_position)
  {
    robot_position_ = robot_position;
  }
  void SetViewPointManager(std::shared_ptr<viewpoint_manager_ns::ViewPointManager> const& viewpoint_manager)
  {
    viewpoint_manager_ = viewpoint_manager;
    use_frontier_ = viewpoint_manager_->UseFrontier();
  }

  // Local coverage
  void SetLookAheadPoint(const Eigen::Vector3d& lookahead_point)
  {
    lookahead_point_ = lookahead_point;
    lookahead_point_update_ = true;
  }
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

  bool IsLocalCoverageComplete()
  {
    return local_coverage_complete_;
  }

  // Visualization
  void GetSelectedViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud);

private:
  int GetBoundaryViewpointIndex(const exploration_path_ns::ExplorationPath& global_path);
  void GetBoundaryViewpointIndices(exploration_path_ns::ExplorationPath global_path);
  void GetNavigationViewPointIndices(exploration_path_ns::ExplorationPath global_path,
                                     std::vector<int>& navigation_viewpoint_indices);
  void UpdateViewPointCoveredPoint(std::vector<bool>& point_list, int viewpoint_index, bool use_array_ind = false);
  void UpdateViewPointCoveredFrontierPoint(std::vector<bool>& frontier_point_list, int viewpoint_index,
                                           bool use_array_ind = false);

  void EnqueueViewpointCandidates(std::vector<std::pair<int, int>>& cover_point_queue,
                                  std::vector<std::pair<int, int>>& frontier_queue,
                                  const std::vector<bool>& covered_point_list,
                                  const std::vector<bool>& covered_frontier_point_list,
                                  const std::vector<int>& selected_viewpoint_array_indices);

  void SelectViewPoint(const std::vector<std::pair<int, int>>& queue, const std::vector<bool>& covered,
                       std::vector<int>& selected_viewpoint_indices, bool use_frontier = false);
  void SelectViewPointFromFrontierQueue(std::vector<std::pair<int, int>>& frontier_queue,
                                        std::vector<bool>& frontier_covered,
                                        std::vector<int>& selected_viewpoint_indices);
  exploration_path_ns::ExplorationPath SolveTSP(const std::vector<int>& selected_viewpoint_indices,
                                                std::vector<int>& ordered_viewpoint_indices);

  // viewpoint_manager_ns::ViewPointManager::Ptr viewpoint_manager_;
  static bool SortPairInRev(const std::pair<int, int>& a, const std::pair<int, int>& b)
  {
    return (a.first > b.first);
  }

  LocalCoveragePlannerParameter parameters_;
  std::shared_ptr<viewpoint_manager_ns::ViewPointManager> viewpoint_manager_;

  Eigen::Vector3d robot_position_;
  Eigen::Vector3d lookahead_point_;

  bool lookahead_point_update_;
  bool use_frontier_;
  bool local_coverage_complete_;

  // Viewpoint indices
  int robot_viewpoint_ind_;
  int start_viewpoint_ind_;
  int end_viewpoint_ind_;
  int lookahead_viewpoint_ind_;

  // Runtime
  int find_path_runtime_;
  int viewpoint_sampling_runtime_;
  int tsp_runtime_;
  static const std::string kRuntimeUnit;

  std::vector<int> last_selected_viewpoint_indices_;
  std::vector<int> last_selected_viewpoint_array_indices_;
};
}  // namespace local_coverage_planner_ns