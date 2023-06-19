/**
 * @file exploration_path.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements the exploration path
 * @version 0.1
 * @date 2020-10-22
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <memory>
#include <vector>

#include <Eigen/Core>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <utils/misc_utils.h>
#include <utils/pointcloud_utils.h>

namespace exploration_path_ns
{
enum class NodeType
{
  ROBOT = 0,
  LOOKAHEAD_POINT = 2,
  LOCAL_VIEWPOINT = 4,
  LOCAL_PATH_START = 6,
  LOCAL_PATH_END = 8,
  LOCAL_VIA_POINT = 10,
  GLOBAL_VIEWPOINT = 1,
  GLOBAL_VIA_POINT = 3,
  HOME = 5
};
struct Node
{
  NodeType type_;
  Eigen::Vector3d position_;
  int local_viewpoint_ind_;
  int keypose_graph_node_ind_;
  int global_subspace_index_;
  bool nonstop_;
  explicit Node();
  explicit Node(Eigen::Vector3d position);
  explicit Node(geometry_msgs::msg::Point point, NodeType type);
  ~Node() = default;
  bool IsLocal();
  friend bool operator==(const Node& n1, const Node& n2);
  friend bool operator!=(const Node& n1, const Node& n2);
};
struct ExplorationPath
{
  std::vector<Node> nodes_;
  double GetLength() const;
  int GetNodeNum() const
  {
    return nodes_.size();
  }
  void Append(const Node& node);
  void Append(const ExplorationPath& path);
  void Reverse();
  nav_msgs::Path GetPath() const;
  void FromPath(const nav_msgs::Path& path);
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const;
  void GetKeyPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr vis_cloud) const;
  void GetNodePositions(std::vector<Eigen::Vector3d>& positions) const;
  void Reset();
};
}  // namespace exploration_path_ns
