/**
 * @file grid_world.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid world
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <vector>
#include <memory>

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <grid/grid.h>
#include <tsp_solver/tsp_solver.h>
#include <keypose_graph/keypose_graph.h>
#include <exploration_path/exploration_path.h>

namespace viewpoint_manager_ns
{
class ViewPointManager;
}

namespace grid_world_ns
{
enum class CellStatus
{
  UNSEEN = 0,
  EXPLORING = 1,
  COVERED = 2,
  COVERED_BY_OTHERS = 3,
  NOGO = 4
};

class Cell
{
public:
  explicit Cell(double x = 0.0, double y = 0.0, double z = 0.0);
  explicit Cell(const geometry_msgs::Point& center);
  ~Cell() = default;
  bool IsCellConnected(int cell_ind);
  void AddViewPoint(int viewpoint_ind)
  {
    viewpoint_indices_.push_back(viewpoint_ind);
  }
  void AddGraphNode(int node_ind)
  {
    keypose_graph_node_indices_.push_back(node_ind);
  }
  void AddConnectedCell(int cell_ind)
  {
    connected_cell_indices_.push_back(cell_ind);
    misc_utils_ns::UniquifyIntVector(connected_cell_indices_);
  }
  void ClearViewPointIndices()
  {
    viewpoint_indices_.clear();
  }
  void ClearGraphNodeIndices()
  {
    keypose_graph_node_indices_.clear();
  }
  void ClearConnectedCellIndices()
  {
    connected_cell_indices_.clear();
  }
  CellStatus GetStatus()
  {
    return status_;
  }
  void SetStatus(CellStatus status)
  {
    status_ = status;
  }
  std::vector<int> GetViewPointIndices()
  {
    return viewpoint_indices_;
  }
  std::vector<int> GetConnectedCellIndices()
  {
    return connected_cell_indices_;
  }
  std::vector<int> GetGraphNodeIndices()
  {
    return keypose_graph_node_indices_;
  }
  geometry_msgs::Point GetPosition()
  {
    return center_;
  }
  void SetPosition(const geometry_msgs::Point& position)
  {
    center_ = position;
  }
  void SetRobotPosition(const geometry_msgs::Point& robot_position)
  {
    robot_position_ = robot_position;
    robot_position_set_ = true;
  }
  void SetKeyposeID(int keypose_id)
  {
    keypose_id_ = keypose_id;
    robot_position_set_ = true;
  }
  bool IsRobotPositionSet()
  {
    return robot_position_set_;
  }
  geometry_msgs::Point GetRobotPosition()
  {
    return robot_position_;
  }
  void AddVisitCount()
  {
    visit_count_++;
  }
  int GetVisitCount()
  {
    return visit_count_;
  }
  void Reset();
  int GetKeyposeID()
  {
    return keypose_id_;
  }
  void SetViewPointPosition(const Eigen::Vector3d& position)
  {
    viewpoint_position_ = position;
  }
  Eigen::Vector3d GetViewPointPosition()
  {
    return viewpoint_position_;
  }
  void SetRoadmapConnectionPoint(const Eigen::Vector3d& roadmap_connection_point)
  {
    roadmap_connection_point_ = roadmap_connection_point;
  }
  Eigen::Vector3d GetRoadmapConnectionPoint()
  {
    return roadmap_connection_point_;
  }
  nav_msgs::Path GetPathToKeyposeGraph()
  {
    return path_to_keypose_graph_;
  }
  void SetPathToKeyposeGraph(const nav_msgs::Path& path)
  {
    path_to_keypose_graph_ = path;
  }
  bool IsPathAddedToKeyposeGraph()
  {
    return path_added_to_keypose_graph_;
  }
  void SetPathAddedToKeyposeGraph(bool add_path)
  {
    path_added_to_keypose_graph_ = add_path;
  }
  bool IsRoadmapConnectionPointSet()
  {
    return roadmap_connection_point_set_;
  }
  void SetRoadmapConnectionPointSet(bool set)
  {
    roadmap_connection_point_set_ = set;
  }

private:
  CellStatus status_;
  // The center location of this cell.
  geometry_msgs::Point center_;
  // Position of the robot where this cell is first observed and turned EXPLORING
  geometry_msgs::Point robot_position_;
  // Whether the robot position has been set for this cell
  bool robot_position_set_;
  // Number of times the cell is visited by the robot
  int visit_count_;
  // Indices of the viewpoints within this cell.
  std::vector<int> viewpoint_indices_;
  // Indices of other cells that are connected by a path.
  std::vector<int> connected_cell_indices_;
  // Indices of connected keypose graph nodes
  std::vector<int> keypose_graph_node_indices_;
  // Whether this cell is in the planning horizon, which consists of nine cells around the robot.
  bool in_horizon_;
  // ID of the keypose where viewpoints in this cell can be observed
  int keypose_id_;
  // Position of the highest score viewpoint
  Eigen::Vector3d viewpoint_position_;
  // Position for connecting the cell to the global roadmap
  Eigen::Vector3d roadmap_connection_point_;
  // Path to the nearest keypose on the keypose graph
  nav_msgs::Path path_to_keypose_graph_;
  // If the path has been added to the keypose graph
  bool path_added_to_keypose_graph_;
  // If the roadmap connection point has been added to the cell
  bool roadmap_connection_point_set_;
};

class GridWorld
{
public:
  explicit GridWorld(ros::NodeHandle& nh);
  explicit GridWorld(int row_num = 1, int col_num = 1, int level_num = 1, double cell_size = 6.0,
                     double cell_height = 6.0, int nearby_grid_num = 5);
  ~GridWorld() = default;
  void ReadParameters(ros::NodeHandle& nh);
  void UpdateNeighborCells(const geometry_msgs::Point& robot_position);
  void UpdateRobotPosition(const geometry_msgs::Point& robot_position);
  void UpdateCellKeyposeGraphNodes(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  int GetMinAddPointNum()
  {
    return kMinAddPointNumSmall;
  }
  int GetMinAddFrontierPointNum()
  {
    return kMinAddFrontierPointNum;
  }
  geometry_msgs::Point GetOrigin()
  {
    // return origin_;
    Eigen::Vector3d origin = subspaces_->GetOrigin();
    geometry_msgs::Point geo_origin;
    geo_origin.x = origin.x();
    geo_origin.y = origin.y();
    geo_origin.z = origin.z();
    return geo_origin;
  }
  int sub2ind(const Eigen::Vector3i& sub)
  {
    return subspaces_->Sub2Ind(sub);
  }
  int sub2ind(int row_idx, int col_idx, int level_idx)
  {
    return subspaces_->Sub2Ind(row_idx, col_idx, level_idx);
  }
  Eigen::Vector3i ind2sub(int ind)
  {
    return subspaces_->Ind2Sub(ind);
  }
  void ind2sub(int ind, int& row_idx, int& col_idx, int& level_idx)
  {
    Eigen::Vector3i sub = subspaces_->Ind2Sub(ind);
    row_idx = sub.x();
    col_idx = sub.y();
    level_idx = sub.z();
  }
  bool SubInBound(const Eigen::Vector3i& sub)
  {
    return subspaces_->InRange(sub);
  }
  bool SubInBound(int row_idx, int col_idx, int level_idx)
  {
    return subspaces_->InRange(Eigen::Vector3i(row_idx, col_idx, level_idx));
  }
  bool IndInBound(int ind)
  {
    return subspaces_->InRange(ind);
  }
  // Get the cell index where the robot is currently in.
  bool AreNeighbors(int cell_ind1, int cell_ind2);
  int GetCellInd(double qx, double qy, double qz);
  void GetCellSub(int& row_idx, int& col_idx, int& level_idx, double qx, double qy, double qz);
  Eigen::Vector3i GetCellSub(const Eigen::Vector3d& point);
  // Get the visualization markers for Rviz display.
  void GetMarker(visualization_msgs::Marker& marker);
  // Get the visualization pointcloud for debugging purpose
  void GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud);

  bool Initialized()
  {
    return initialized_;
  }
  void SetUseKeyposeGraph(bool use_keypose_graph)
  {
    use_keypose_graph_ = use_keypose_graph;
  }
  bool UseKeyposeGraph()
  {
    return use_keypose_graph_;
  }

  void AddViewPointToCell(int cell_ind, int viewpoint_ind);
  void AddGraphNodeToCell(int cell_ind, int node_ind);
  void ClearCellViewPointIndices(int cell_ind);
  std::vector<int> GetCellViewPointIndices(int cell_ind);
  std::vector<int> GetNeighborCellIndices()
  {
    return neighbor_cell_indices_;
  };
  void GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, const Eigen::Vector3i& neighbor_range,
                              std::vector<int>& neighbor_indices);
  void GetNeighborCellIndices(const geometry_msgs::Point& position, const Eigen::Vector3i& neighbor_range,
                              std::vector<int>& neighbor_indices);
  void GetExploringCellIndices(std::vector<int>& exploring_cell_indices);
  CellStatus GetCellStatus(int cell_ind);
  void SetCellStatus(int cell_ind, CellStatus status);
  geometry_msgs::Point GetCellPosition(int cell_ind);
  void SetCellRobotPosition(int cell_ind, const geometry_msgs::Point& robot_position);
  geometry_msgs::Point GetCellRobotPosition(int cell_ind);
  void CellAddVisitCount(int cell_ind);
  int GetCellVisitCount(int cell_ind);
  bool IsRobotPositionSet(int cell_ind);
  void Reset();
  int GetCellStatusCount(grid_world_ns::CellStatus status);
  void UpdateCellStatus(const std::unique_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager);
  exploration_path_ns::ExplorationPath
  SolveGlobalTSP(const std::unique_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                 std::vector<int>& ordered_cell_indices,
                 const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph = nullptr);

  inline void SetCurKeyposeGraphNodeInd(int node_ind)
  {
    cur_keypose_graph_node_ind_ = node_ind;
  }
  inline void SetCurKeyposeGraphNodePosition(geometry_msgs::Point node_position)
  {
    cur_keypose_graph_node_position_ = node_position;
  }
  inline void SetCurKeyposeID(int keypose_id)
  {
    cur_keypose_id_ = keypose_id;
  }

  inline void SetCurKeypose(const Eigen::Vector3d& cur_keypose)
  {
    cur_keypose_ = cur_keypose;
  }
  int GetCellKeyposeID(int cell_ind);
  void SetHomePosition(const Eigen::Vector3d& home_position)
  {
    home_position_ = home_position;
    set_home_ = true;
  }
  bool HomeSet()
  {
    return set_home_;
  }
  bool IsReturningHome()
  {
    return return_home_;
  }
  void GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions);
  void AddPathsToKeyposeGraph(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  void AddPathsInBetweenCells(const std::unique_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                              const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph);
  bool PathValid(const nav_msgs::Path& path, int from_cell_ind, int to_cell_ind);
  bool HasDirectKeyposeGraphConnection(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                       const Eigen::Vector3d& start_position, const Eigen::Vector3d& goal_position);

private:
  int kRowNum;
  int kColNum;
  int kLevelNum;
  double kCellSize;
  double kCellHeight;
  int KNearbyGridNum;
  int kMinAddPointNumSmall;
  int kMinAddPointNumBig;
  int kMinAddFrontierPointNum;
  int kCellExploringToCoveredThr;
  int kCellCoveredToExploringThr;
  int kCellExploringToAlmostCoveredThr;
  int kCellAlmostCoveredToExploringThr;
  int kCellUnknownToExploringThr;

  std::vector<Cell> cells_;
  std::unique_ptr<grid_ns::Grid<Cell>> subspaces_;
  bool initialized_;
  bool use_keypose_graph_;
  int cur_keypose_id_;
  geometry_msgs::Point robot_position_;
  geometry_msgs::Point origin_;
  std::vector<int> neighbor_cell_indices_;
  std::vector<int> almost_covered_cell_indices_;
  std::vector<std::pair<int, int>> to_connect_cell_indices_;
  std::vector<nav_msgs::Path> to_connect_cell_paths_;
  Eigen::Vector3d home_position_;
  Eigen::Vector3d cur_keypose_;
  bool set_home_;
  bool return_home_;
  geometry_msgs::Point cur_keypose_graph_node_position_;
  int cur_keypose_graph_node_ind_;
  int cur_robot_cell_ind_;
  int prev_robot_cell_ind_;
};
}  // namespace grid_world_ns
