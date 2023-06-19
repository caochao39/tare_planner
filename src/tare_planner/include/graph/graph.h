/**
 * @file graph.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a graph
 * @version 0.1
 * @date 2021-07-11
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <cmath>
#include <Eigen/Core>

#include <nav_msgs/msg/path.hpp>

namespace tare
{
class Graph
{
public:
  explicit Graph(int node_number);
  ~Graph() = default;

  void AddNode(const Eigen::Vector3d& position);
  void SetNodePosition(int node_index, const Eigen::Vector3d& position);
  void AddOneWayEdge(int from_node_index, int to_node_index, double distance);
  void AddTwoWayEdge(int from_node_index, int to_node_index, double distance);
  double GetShortestPath(int from_node_index, int to_node_index, bool get_path, nav_msgs::Path& shortest_path,
                         std::vector<int>& node_indices);

private:
  bool NodeIndexInRange(int node_index)
  {
    return node_index >= 0 && node_index < connection_.size();
  }
  double AStarSearch(int from_node_index, int to_node_index, bool get_path, std::vector<int>& node_indices);
  // Node connectivity
  std::vector<std::vector<int>> connection_;
  // Distances between two nodes
  std::vector<std::vector<double>> distance_;
  // Node positions
  std::vector<Eigen::Vector3d> positions_;
};
}  // namespace tare
