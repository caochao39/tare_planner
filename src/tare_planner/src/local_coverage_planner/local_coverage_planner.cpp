/**
 * @file local_coverage_planner.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that ensures coverage in the surroundings of the robot
 * @version 0.1
 * @date 2021-05-31
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "local_coverage_planner/local_coverage_planner.h"

namespace local_coverage_planner_ns
{
const std::string LocalCoveragePlanner::kRuntimeUnit = "us";

bool LocalCoveragePlannerParameter::ReadParameters(ros::NodeHandle& nh)
{
  kMinAddPointNum = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumSmall", 60);
  kMinAddFrontierPointNum = misc_utils_ns::getParam<int>(nh, "kMinAddFrontierPointNum", 30);
  kGreedyViewPointSampleRange = misc_utils_ns::getParam<int>(nh, "kGreedyViewPointSampleRange", 5);
  kLocalPathOptimizationItrMax = misc_utils_ns::getParam<int>(nh, "kLocalPathOptimizationItrMax", 10);

  return true;
}
LocalCoveragePlanner::LocalCoveragePlanner(ros::NodeHandle& nh)
  : lookahead_point_update_(false), use_frontier_(true), local_coverage_complete_(false)
{
  parameters_.ReadParameters(nh);
}

int LocalCoveragePlanner::GetBoundaryViewpointIndex(const exploration_path_ns::ExplorationPath& global_path)
{
  int boundary_viewpoint_index = robot_viewpoint_ind_;
  if (!global_path.nodes_.empty())
  {
    if (viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_.front().position_))
    {
      for (int i = 0; i < global_path.nodes_.size(); i++)
      {
        if (global_path.nodes_[i].type_ == exploration_path_ns::NodeType::GLOBAL_VIEWPOINT ||
            global_path.nodes_[i].type_ == exploration_path_ns::NodeType::HOME ||
            !viewpoint_manager_->InLocalPlanningHorizon(global_path.nodes_[i].position_))
        {
          break;
        }
        boundary_viewpoint_index = viewpoint_manager_->GetNearestCandidateViewPointInd(global_path.nodes_[i].position_);
      }
    }
  }
  return boundary_viewpoint_index;
}

void LocalCoveragePlanner::GetBoundaryViewpointIndices(exploration_path_ns::ExplorationPath global_path)
{
  start_viewpoint_ind_ = GetBoundaryViewpointIndex(global_path);
  global_path.Reverse();
  end_viewpoint_ind_ = GetBoundaryViewpointIndex(global_path);
}

void LocalCoveragePlanner::GetNavigationViewPointIndices(exploration_path_ns::ExplorationPath global_path,
                                                         std::vector<int>& navigation_viewpoint_indices)
{
  // Get start and end point
  robot_viewpoint_ind_ = viewpoint_manager_->GetNearestCandidateViewPointInd(robot_position_);
  lookahead_viewpoint_ind_ = viewpoint_manager_->GetNearestCandidateViewPointInd(lookahead_point_);
  if (!lookahead_point_update_ || !viewpoint_manager_->InRange(lookahead_viewpoint_ind_))
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

void LocalCoveragePlanner::UpdateViewPointCoveredPoint(std::vector<bool>& point_list, int viewpoint_index,
                                                       bool use_array_ind)
{
  for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(point_list, point_ind));
    point_list[point_ind] = true;
  }
}
void LocalCoveragePlanner::UpdateViewPointCoveredFrontierPoint(std::vector<bool>& frontier_point_list,
                                                               int viewpoint_index, bool use_array_ind)
{
  for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredFrontierPointList(viewpoint_index, use_array_ind))
  {
    MY_ASSERT(misc_utils_ns::InRange<bool>(frontier_point_list, point_ind));
    frontier_point_list[point_ind] = true;
  }
}

void LocalCoveragePlanner::EnqueueViewpointCandidates(std::vector<std::pair<int, int>>& cover_point_queue,
                                                      std::vector<std::pair<int, int>>& frontier_queue,
                                                      const std::vector<bool>& covered_point_list,
                                                      const std::vector<bool>& covered_frontier_point_list,
                                                      const std::vector<int>& selected_viewpoint_array_indices)
{
  for (const auto& viewpoint_index : viewpoint_manager_->GetViewPointCandidateIndices())
  {
    if (viewpoint_manager_->ViewPointVisited(viewpoint_index) ||
        !viewpoint_manager_->ViewPointInExploringCell(viewpoint_index))
    {
      continue;
    }
    int viewpoint_array_index = viewpoint_manager_->GetViewPointArrayInd(viewpoint_index);
    if (std::find(selected_viewpoint_array_indices.begin(), selected_viewpoint_array_indices.end(),
                  viewpoint_array_index) != selected_viewpoint_array_indices.end())
    {
      continue;
    }
    int covered_point_num =
        viewpoint_manager_->GetViewPointCoveredPointNum(covered_point_list, viewpoint_array_index, true);
    if (covered_point_num >= parameters_.kMinAddPointNum)
    {
      cover_point_queue.emplace_back(covered_point_num, viewpoint_index);
    }
    else if (use_frontier_)
    {
      int covered_frontier_point_num = viewpoint_manager_->GetViewPointCoveredFrontierPointNum(
          covered_frontier_point_list, viewpoint_array_index, true);
      if (covered_frontier_point_num >= parameters_.kMinAddFrontierPointNum)
      {
        frontier_queue.emplace_back(covered_frontier_point_num, viewpoint_index);
      }
    }
  }

  // Sort the queue
  std::sort(cover_point_queue.begin(), cover_point_queue.end(), SortPairInRev);
  if (use_frontier_)
  {
    std::sort(frontier_queue.begin(), frontier_queue.end(), SortPairInRev);
  }
}

void LocalCoveragePlanner::SelectViewPoint(const std::vector<std::pair<int, int>>& queue,
                                           const std::vector<bool>& covered,
                                           std::vector<int>& selected_viewpoint_indices, bool use_frontier)
{
  if (use_frontier)
  {
    if (queue.empty() || queue[0].first < parameters_.kMinAddFrontierPointNum)
    {
      return;
    }
  }
  else
  {
    if (queue.empty() || queue[0].first < parameters_.kMinAddPointNum)
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
      if (queue_copy[i].first >= parameters_.kMinAddFrontierPointNum)
      {
        sample_range++;
      }
    }
    else
    {
      if (queue_copy[i].first >= parameters_.kMinAddPointNum)
      {
        sample_range++;
      }
    }
  }

  sample_range = std::min(parameters_.kGreedyViewPointSampleRange, sample_range);
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);
  int queue_idx = gen_next_queue_idx(gen);
  int cur_ind = queue_copy[queue_idx].second;

  while (true)
  {
    int cur_array_ind = viewpoint_manager_->GetViewPointArrayInd(cur_ind);
    if (use_frontier)
    {
      for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredFrontierPointList(cur_array_ind, true))

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
      for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredPointList(cur_array_ind, true))
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
      int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
      if (use_frontier)
      {
        for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredFrontierPointList(array_ind, true))
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
        for (const auto& point_ind : viewpoint_manager_->GetViewPointCoveredPointList(array_ind, true))
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

    if (queue_copy.empty() || queue_copy[0].first < parameters_.kMinAddPointNum)
    {
      break;
    }
    if (use_frontier)
    {
      if (queue_copy.empty() || queue_copy[0].first < parameters_.kMinAddFrontierPointNum)
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
        if (queue[i].first >= parameters_.kMinAddFrontierPointNum)
        {
          sample_range++;
        }
      }
      else
      {
        if (queue[i].first >= parameters_.kMinAddPointNum)
        {
          sample_range++;
        }
      }
    }
    sample_range = std::min(parameters_.kGreedyViewPointSampleRange, sample_range);
    std::uniform_int_distribution<int> gen_next_queue_idx(0, sample_range - 1);
    queue_idx = gen_next_queue_idx(gen);
    cur_ind = queue_copy[queue_idx].second;
  }
}

void LocalCoveragePlanner::SelectViewPointFromFrontierQueue(std::vector<std::pair<int, int>>& frontier_queue,
                                                            std::vector<bool>& frontier_covered,
                                                            std::vector<int>& selected_viewpoint_indices)
{
  if (use_frontier_ && !frontier_queue.empty() && frontier_queue[0].first > parameters_.kMinAddFrontierPointNum)
  {
    // Update the frontier queue
    for (const auto& ind : selected_viewpoint_indices)
    {
      UpdateViewPointCoveredFrontierPoint(frontier_covered, ind);
    }
    for (int i = 0; i < frontier_queue.size(); i++)
    {
      int ind = frontier_queue[i].second;
      int covered_frontier_point_num = viewpoint_manager_->GetViewPointCoveredFrontierPointNum(frontier_covered, ind);
      frontier_queue[i].first = covered_frontier_point_num;
    }
    std::sort(frontier_queue.begin(), frontier_queue.end(), SortPairInRev);
    SelectViewPoint(frontier_queue, frontier_covered, selected_viewpoint_indices, true);
  }
}

exploration_path_ns::ExplorationPath LocalCoveragePlanner::SolveTSP(const std::vector<int>& selected_viewpoint_indices,
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
    // int from_graph_idx = graph_index_map_[from_ind];
    for (int j = 0; j < i; j++)
    {
      int to_ind = selected_viewpoint_indices[j];
      nav_msgs::Path path = viewpoint_manager_->GetViewPointShortestPath(from_ind, to_ind);
      double path_length = misc_utils_ns::GetPathLength(path);
      //   int to_graph_idx = graph_index_map_[to_ind];
      //   double path_length =
      //       misc_utils_ns::AStarSearch(candidate_viewpoint_graph_, candidate_viewpoint_dist_,
      //                                  candidate_viewpoint_position_, from_graph_idx, to_graph_idx, false, tmp);
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

      //   from_graph_idx = graph_index_map_[cur_ind];
      //   to_graph_idx = graph_index_map_[next_ind];

      // Add viewpoint node
      // int cur_array_ind = grid_->GetArrayInd(cur_ind);
      // geometry_msgs::Point cur_node_position = viewpoints_[cur_array_ind].GetPosition();
      geometry_msgs::Point cur_node_position = viewpoint_manager_->GetViewPointPosition(cur_ind);
      exploration_path_ns::Node cur_node(cur_node_position, exploration_path_ns::NodeType::LOCAL_VIEWPOINT);
      cur_node.local_viewpoint_ind_ = cur_ind;
      if (cur_ind == robot_viewpoint_ind_)
      {
        cur_node.type_ = exploration_path_ns::NodeType::ROBOT;
      }
      else if (cur_ind == lookahead_viewpoint_ind_)
      {
        int covered_point_num = viewpoint_manager_->GetViewPointCoveredPointNum(cur_ind);
        int covered_frontier_num = viewpoint_manager_->GetViewPointCoveredFrontierPointNum(cur_ind);
        if (covered_point_num > parameters_.kMinAddPointNum ||
            covered_frontier_num > parameters_.kMinAddFrontierPointNum)
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

      nav_msgs::Path path_between_viewpoints = viewpoint_manager_->GetViewPointShortestPath(cur_ind, next_ind);

      //   std::vector<int> path_graph_indices;
      //   misc_utils_ns::AStarSearch(candidate_viewpoint_graph_, candidate_viewpoint_dist_,
      //   candidate_viewpoint_position_,
      //                              from_graph_idx, to_graph_idx, true, path_graph_indices);
      // Add viapoint nodes;
      //   if (path_graph_indices.size() > 2)
      if (path_between_viewpoints.poses.size() > 2)
      {
        // for (int j = 1; j < path_graph_indices.size() - 1; j++)
        for (int j = 1; j < path_between_viewpoints.poses.size() - 1; j++)
        {
          //   int graph_idx = path_graph_indices[j];
          //   int ind = candidate_indices_[graph_idx];
          exploration_path_ns::Node node;
          node.type_ = exploration_path_ns::NodeType::LOCAL_VIA_POINT;
          node.local_viewpoint_ind_ = -1;
          //   geometry_msgs::Point node_position = viewpoint_manager_->GetViewPointPosition(ind);
          //   node.position_.x() = node_position.x;
          //   node.position_.y() = node_position.y;
          //   node.position_.z() = node_position.z;
          node.position_.x() = path_between_viewpoints.poses[j].pose.position.x;
          node.position_.y() = path_between_viewpoints.poses[j].pose.position.y;
          node.position_.z() = path_between_viewpoints.poses[j].pose.position.z;
          tsp_path.Append(node);
        }
      }

      geometry_msgs::Point next_node_position = viewpoint_manager_->GetViewPointPosition(next_ind);
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

exploration_path_ns::ExplorationPath LocalCoveragePlanner::SolveLocalCoverageProblem(
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
    if (viewpoint_manager_->ViewPointVisited(viewpoint_array_ind, true) ||
        !viewpoint_manager_->IsViewPointCandidate(viewpoint_array_ind, true))
    {
      continue;
    }
    int covered_point_num = viewpoint_manager_->GetViewPointCoveredPointNum(covered, viewpoint_array_ind, true);
    if (covered_point_num >= parameters_.kMinAddPointNum)
    {
      reused_viewpoint_indices.push_back(viewpoint_manager_->GetViewPointInd(viewpoint_array_ind));
    }
    else if (use_frontier_)
    {
      int covered_frontier_point_num =
          viewpoint_manager_->GetViewPointCoveredFrontierPointNum(frontier_covered, viewpoint_array_ind, true);
      if (covered_frontier_point_num >= parameters_.kMinAddFrontierPointNum)
      {
        reused_viewpoint_indices.push_back(viewpoint_manager_->GetViewPointInd(viewpoint_array_ind));
      }
    }
  }

  for (const auto& ind : reused_viewpoint_indices)
  {
    int viewpoint_array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
    pre_selected_viewpoint_array_indices.push_back(viewpoint_array_ind);
  }
  for (const auto& ind : navigation_viewpoint_indices)
  {
    int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
    pre_selected_viewpoint_array_indices.push_back(array_ind);
  }

  // Update coverage
  for (auto& viewpoint_array_ind : pre_selected_viewpoint_array_indices)
  {
    // Update covered points and frontiers
    UpdateViewPointCoveredPoint(covered, viewpoint_array_ind, true);
    if (use_frontier_)
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
  if (!queue.empty() && queue[0].first > parameters_.kMinAddPointNum)
  {
    double min_path_length = DBL_MAX;
    for (int itr = 0; itr < parameters_.kLocalPathOptimizationItrMax; itr++)
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
    int array_ind = viewpoint_manager_->GetViewPointArrayInd(ind);
    last_selected_viewpoint_array_indices_.push_back(array_ind);
  }

  int viewpoint_num = viewpoint_manager_->GetViewPointNum();
  for (int i = 0; i < viewpoint_num; i++)
  {
    viewpoint_manager_->SetViewPointSelected(i, false, true);
  }
  for (const auto& viewpoint_index : last_selected_viewpoint_indices_)
  {
    if (viewpoint_index != robot_viewpoint_ind_ && viewpoint_index != start_viewpoint_ind_ &&
        viewpoint_index != end_viewpoint_ind_ && viewpoint_index != lookahead_viewpoint_ind_)
    {
      viewpoint_manager_->SetViewPointSelected(viewpoint_index, true);
    }
  }
  return local_path;
}

void LocalCoveragePlanner::GetSelectedViewPointVisCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud)
{
  cloud->clear();
  for (const auto& viewpoint_index : last_selected_viewpoint_indices_)
  {
    geometry_msgs::Point position = viewpoint_manager_->GetViewPointPosition(viewpoint_index);
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

}  // namespace local_coverage_planner_ns