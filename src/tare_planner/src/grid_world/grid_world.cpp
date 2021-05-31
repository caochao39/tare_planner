/**
 * @file grid_world.cpp
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that implements a grid world
 * @version 0.1
 * @date 2019-11-06
 *
 * @copyright Copyright (c) 2021
 *
 */

#include "../../include/grid_world/grid_world.h"
#include <map>
#include <algorithm>
#include <utils/misc_utils.h>
#include <viewpoint_manager/viewpoint_manager.h>

namespace grid_world_ns
{
Cell::Cell(double x, double y, double z)
  : in_horizon_(false)
  , robot_position_set_(false)
  , visit_count_(0)
  , keypose_id_(0)
  , path_added_to_keypose_graph_(false)
  , roadmap_connection_point_set_(false)
  , viewpoint_position_(Eigen::Vector3d(x, y, z))
  , roadmap_connection_point_(Eigen::Vector3d(x, y, z))
{
  center_.x = x;
  center_.y = y;
  center_.z = z;

  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;
  status_ = CellStatus::UNSEEN;
}

Cell::Cell(const geometry_msgs::Point& center) : Cell(center.x, center.y, center.z)
{
}

void Cell::Reset()
{
  status_ = CellStatus::UNSEEN;
  robot_position_.x = 0;
  robot_position_.y = 0;
  robot_position_.z = 0;
  visit_count_ = 0;
  viewpoint_indices_.clear();
  connected_cell_indices_.clear();
  keypose_graph_node_indices_.clear();
}

bool Cell::IsCellConnected(int cell_ind)
{
  if (std::find(connected_cell_indices_.begin(), connected_cell_indices_.end(), cell_ind) !=
      connected_cell_indices_.end())
  {
    return true;
  }
  else
  {
    return false;
  }
}

GridWorld::GridWorld(ros::NodeHandle& nh) : initialized_(false), use_keypose_graph_(false)
{
  ReadParameters(nh);
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  origin_.x = 0.0;
  origin_.y = 0.0;
  origin_.z = 0.0;

  Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum);
  Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
  Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
  Cell cell_tmp;
  subspaces_ = std::make_unique<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    subspaces_->GetCell(i) = grid_world_ns::Cell();
  }

  home_position_.x() = 0.0;
  home_position_.y() = 0.0;
  home_position_.z() = 0.0;

  cur_keypose_graph_node_position_.x = 0.0;
  cur_keypose_graph_node_position_.y = 0.0;
  cur_keypose_graph_node_position_.z = 0.0;

  set_home_ = false;
  return_home_ = false;

  cur_robot_cell_ind_ = -1;
  prev_robot_cell_ind_ = -1;
}

GridWorld::GridWorld(int row_num, int col_num, int level_num, double cell_size, double cell_height, int nearby_grid_num)
  : kRowNum(row_num)
  , kColNum(col_num)
  , kLevelNum(level_num)
  , kCellSize(cell_size)
  , kCellHeight(cell_height)
  , KNearbyGridNum(nearby_grid_num)
  , kMinAddPointNumSmall(60)
  , kMinAddPointNumBig(100)
  , kMinAddFrontierPointNum(30)
  , kCellExploringToCoveredThr(1)
  , kCellCoveredToExploringThr(10)
  , kCellExploringToAlmostCoveredThr(10)
  , kCellAlmostCoveredToExploringThr(20)
  , kCellUnknownToExploringThr(1)
  , cur_keypose_id_(0)
  , cur_keypose_graph_node_ind_(0)
  , cur_robot_cell_ind_(-1)
  , prev_robot_cell_ind_(-1)
  , cur_keypose_(0, 0, 0)
  , initialized_(false)
  , use_keypose_graph_(false)
{
  robot_position_.x = 0.0;
  robot_position_.y = 0.0;
  robot_position_.z = 0.0;

  origin_.x = 0.0;
  origin_.y = 0.0;
  origin_.z = 0.0;

  Eigen::Vector3i grid_size(kRowNum, kColNum, kLevelNum);
  Eigen::Vector3d grid_origin(0.0, 0.0, 0.0);
  Eigen::Vector3d grid_resolution(kCellSize, kCellSize, kCellHeight);
  Cell cell_tmp;
  subspaces_ = std::make_unique<grid_ns::Grid<Cell>>(grid_size, cell_tmp, grid_origin, grid_resolution);
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    subspaces_->GetCell(i) = grid_world_ns::Cell();
  }

  home_position_.x() = 0.0;
  home_position_.y() = 0.0;
  home_position_.z() = 0.0;

  cur_keypose_graph_node_position_.x = 0.0;
  cur_keypose_graph_node_position_.y = 0.0;
  cur_keypose_graph_node_position_.z = 0.0;

  set_home_ = false;
  return_home_ = false;
}

void GridWorld::ReadParameters(ros::NodeHandle& nh)
{
  kRowNum = misc_utils_ns::getParam<int>(nh, "kGridWorldXNum", 121);
  kColNum = misc_utils_ns::getParam<int>(nh, "kGridWorldYNum", 121);
  kLevelNum = misc_utils_ns::getParam<int>(nh, "kGridWorldZNum", 121);
  kCellSize = misc_utils_ns::getParam<double>(nh, "kGridWorldCellSize", 8.0);
  kCellHeight = misc_utils_ns::getParam<double>(nh, "kGridWorldCellHeight", 8.0);
  KNearbyGridNum = misc_utils_ns::getParam<int>(nh, "kGridWorldNearbyGridNum", 5);
  kMinAddPointNumSmall = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumSmall", 60);
  kMinAddPointNumBig = misc_utils_ns::getParam<int>(nh, "kMinAddPointNumBig", 100);
  kMinAddFrontierPointNum = misc_utils_ns::getParam<int>(nh, "kMinAddFrontierPointNum", 30);
  kCellExploringToCoveredThr = misc_utils_ns::getParam<int>(nh, "kCellExploringToCoveredThr", 1);
  kCellCoveredToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellCoveredToExploringThr", 10);
  kCellExploringToAlmostCoveredThr = misc_utils_ns::getParam<int>(nh, "kCellExploringToAlmostCoveredThr", 10);
  kCellAlmostCoveredToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellAlmostCoveredToExploringThr", 20);
  kCellUnknownToExploringThr = misc_utils_ns::getParam<int>(nh, "kCellUnknownToExploringThr", 1);
}

void GridWorld::UpdateNeighborCells(const geometry_msgs::Point& robot_position)
{
  if (!initialized_)
  {
    initialized_ = true;
    origin_.x = robot_position.x - (kCellSize * kRowNum) / 2;
    origin_.y = robot_position.y - (kCellSize * kColNum) / 2;
    origin_.z = robot_position.z - (kCellHeight * kLevelNum) / 2;
    subspaces_->SetOrigin(Eigen::Vector3d(origin_.x, origin_.y, origin_.z));
    // Update cell centers
    for (int i = 0; i < kRowNum; i++)
    {
      for (int j = 0; j < kColNum; j++)
      {
        for (int k = 0; k < kLevelNum; k++)
        {
          Eigen::Vector3d subspace_center_position = subspaces_->Sub2Pos(i, j, k);
          geometry_msgs::Point subspace_center_geo_position;
          subspace_center_geo_position.x = subspace_center_position.x();
          subspace_center_geo_position.y = subspace_center_position.y();
          subspace_center_geo_position.z = subspace_center_position.z();
          subspaces_->GetCell(i, j, k).SetPosition(subspace_center_geo_position);
          subspaces_->GetCell(i, j, k).SetRoadmapConnectionPoint(subspace_center_position);
        }
      }
    }
  }

  // Get neighbor cells
  std::vector<int> prev_neighbor_cell_indices = neighbor_cell_indices_;
  neighbor_cell_indices_.clear();
  int N = KNearbyGridNum / 2;
  int M = 1;
  GetNeighborCellIndices(robot_position, Eigen::Vector3i(N, N, M), neighbor_cell_indices_);

  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    if (std::find(prev_neighbor_cell_indices.begin(), prev_neighbor_cell_indices.end(), cell_ind) ==
        prev_neighbor_cell_indices.end())
    {
      // subspaces_->GetCell(cell_ind).AddVisitCount();
      subspaces_->GetCell(cell_ind).AddVisitCount();
    }
  }
}

void GridWorld::UpdateRobotPosition(const geometry_msgs::Point& robot_position)
{
  robot_position_ = robot_position;
  int robot_cell_ind = GetCellInd(robot_position_.x, robot_position_.y, robot_position_.z);
  if (cur_robot_cell_ind_ != robot_cell_ind)
  {
    prev_robot_cell_ind_ = cur_robot_cell_ind_;
    cur_robot_cell_ind_ = robot_cell_ind;
  }
}

void GridWorld::UpdateCellKeyposeGraphNodes(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  std::vector<int> keypose_graph_connected_node_indices = keypose_graph->GetConnectedGraphNodeIndices();

  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      subspaces_->GetCell(i).ClearGraphNodeIndices();
    }
  }
  for (const auto& node_ind : keypose_graph_connected_node_indices)
  {
    geometry_msgs::Point node_position = keypose_graph->GetNodePosition(node_ind);
    int cell_ind = GetCellInd(node_position.x, node_position.y, node_position.z);
    if (subspaces_->InRange(cell_ind))
    {
      if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
      {
        subspaces_->GetCell(cell_ind).AddGraphNode(node_ind);
      }
    }
  }
}

bool GridWorld::AreNeighbors(int cell_ind1, int cell_ind2)
{
  Eigen::Vector3i cell_sub1 = subspaces_->Ind2Sub(cell_ind1);
  Eigen::Vector3i cell_sub2 = subspaces_->Ind2Sub(cell_ind2);
  Eigen::Vector3i diff = cell_sub1 - cell_sub2;
  if (std::abs(diff.x()) + std::abs(diff.y()) + std::abs(diff.z()) == 1)
  {
    return true;
  }
  else
  {
    return false;
  }
}

int GridWorld::GetCellInd(double qx, double qy, double qz)
{
  Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
  if (subspaces_->InRange(sub))
  {
    return subspaces_->Sub2Ind(sub);
  }
  else
  {
    return -1;
  }
}

void GridWorld::GetCellSub(int& row_idx, int& col_idx, int& level_idx, double qx, double qy, double qz)
{
  Eigen::Vector3i sub = subspaces_->Pos2Sub(qx, qy, qz);
  row_idx = (sub.x() >= 0 && sub.x() < kRowNum) ? sub.x() : -1;
  col_idx = (sub.y() >= 0 && sub.y() < kColNum) ? sub.y() : -1;
  level_idx = (sub.z() >= 0 && sub.z() < kLevelNum) ? sub.z() : -1;
}

Eigen::Vector3i GridWorld::GetCellSub(const Eigen::Vector3d& point)
{
  return subspaces_->Pos2Sub(point);
}

void GridWorld::GetMarker(visualization_msgs::Marker& marker)
{
  marker.points.clear();
  marker.colors.clear();
  marker.scale.x = kCellSize;
  marker.scale.y = kCellSize;
  marker.scale.z = kCellHeight;

  int exploring_count = 0;
  int covered_count = 0;
  int unseen_count = 0;

  for (int i = 0; i < kRowNum; i++)
  {
    for (int j = 0; j < kColNum; j++)
    {
      for (int k = 0; k < kLevelNum; k++)
      {
        int cell_ind = subspaces_->Sub2Ind(i, j, k);
        geometry_msgs::Point cell_center = subspaces_->GetCell(cell_ind).GetPosition();
        std_msgs::ColorRGBA color;
        bool add_marker = false;
        if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN)
        {
          color.r = 0.0;
          color.g = 0.0;
          color.b = 1.0;
          color.a = 0.1;
          unseen_count++;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED)
        {
          color.r = 1.0;
          color.g = 1.0;
          color.b = 0.0;
          color.a = 0.1;
          covered_count++;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING)
        {
          color.r = 0.0;
          color.g = 1.0;
          color.b = 0.0;
          color.a = 0.1;
          exploring_count++;
          add_marker = true;
        }
        else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::NOGO)
        {
          color.r = 1.0;
          color.g = 0.0;
          color.b = 0.0;
          color.a = 0.1;
        }
        else
        {
          color.r = 0.8;
          color.g = 0.8;
          color.b = 0.8;
          color.a = 0.1;
        }
        if (add_marker)
        {
          marker.colors.push_back(color);
          marker.points.push_back(cell_center);
        }
      }
    }
  }
  //        // Color neighbor cells differently
  //        for(const auto & ind : neighbor_cell_indices_){
  //            if(cells_[ind].GetStatus() == CellStatus::UNSEEN) continue;
  //            marker.colors[ind].a = 0.8;
  //        }
}

void GridWorld::GetVisualizationCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr& vis_cloud)
{
  vis_cloud->points.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    CellStatus cell_status = subspaces_->GetCell(i).GetStatus();
    if (!subspaces_->GetCell(i).GetConnectedCellIndices().empty())
    {
      pcl::PointXYZI point;
      Eigen::Vector3d position = subspaces_->GetCell(i).GetRoadmapConnectionPoint();
      point.x = position.x();
      point.y = position.y();
      point.z = position.z();
      point.intensity = i;
      vis_cloud->points.push_back(point);
    }
  }
}

void GridWorld::AddViewPointToCell(int cell_ind, int viewpoint_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddViewPoint(viewpoint_ind);
}

void GridWorld::AddGraphNodeToCell(int cell_ind, int node_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddGraphNode(node_ind);
}

void GridWorld::ClearCellViewPointIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).ClearViewPointIndices();
}

std::vector<int> GridWorld::GetCellViewPointIndices(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetViewPointIndices();
}

void GridWorld::GetNeighborCellIndices(const Eigen::Vector3i& center_cell_sub, const Eigen::Vector3i& neighbor_range,
                                       std::vector<int>& neighbor_indices)
{
  int row_idx = 0;
  int col_idx = 0;
  int level_idx = 0;
  for (int i = -neighbor_range.x(); i <= neighbor_range.x(); i++)
  {
    for (int j = -neighbor_range.y(); j <= neighbor_range.y(); j++)
    {
      row_idx = center_cell_sub.x() + i;
      col_idx = center_cell_sub.y() + j;
      for (int k = -neighbor_range.z(); k <= neighbor_range.z(); k++)
      {
        level_idx = center_cell_sub.z() + k;
        Eigen::Vector3i sub(row_idx, col_idx, level_idx);
        // if (SubInBound(row_idx, col_idx, level_idx))
        if (subspaces_->InRange(sub))
        {
          // int ind = sub2ind(row_idx, col_idx, level_idx);
          int ind = subspaces_->Sub2Ind(sub);
          neighbor_cell_indices_.push_back(ind);
        }
      }
    }
  }
}
void GridWorld::GetNeighborCellIndices(const geometry_msgs::Point& position, const Eigen::Vector3i& neighbor_range,
                                       std::vector<int>& neighbor_indices)
{
  Eigen::Vector3i center_cell_sub = GetCellSub(Eigen::Vector3d(position.x, position.y, position.z));

  GetNeighborCellIndices(center_cell_sub, neighbor_range, neighbor_indices);
}

void GridWorld::GetExploringCellIndices(std::vector<int>& exploring_cell_indices)
{
  exploring_cell_indices.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      exploring_cell_indices.push_back(i);
    }
  }
}

CellStatus GridWorld::GetCellStatus(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetStatus();
}

void GridWorld::SetCellStatus(int cell_ind, grid_world_ns::CellStatus status)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetStatus(status);
}

geometry_msgs::Point GridWorld::GetCellPosition(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetPosition();
}

void GridWorld::SetCellRobotPosition(int cell_ind, const geometry_msgs::Point& robot_position)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position);
}

geometry_msgs::Point GridWorld::GetCellRobotPosition(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetRobotPosition();
}

void GridWorld::CellAddVisitCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  subspaces_->GetCell(cell_ind).AddVisitCount();
}

int GridWorld::GetCellVisitCount(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetVisitCount();
}

bool GridWorld::IsRobotPositionSet(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).IsRobotPositionSet();
}

void GridWorld::Reset()
{
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    subspaces_->GetCell(i).Reset();
  }
}

int GridWorld::GetCellStatusCount(grid_world_ns::CellStatus status)
{
  int count = 0;
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == status)
    {
      count++;
    }
  }
  return count;
}

void GridWorld::UpdateCellStatus(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager)
{
  int exploring_count = 0;
  int unseen_count = 0;
  int covered_count = 0;
  for (int i = 0; i < subspaces_->GetCellNumber(); ++i)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      exploring_count++;
    }
    else if (subspaces_->GetCell(i).GetStatus() == CellStatus::UNSEEN)
    {
      unseen_count++;
    }
    else if (subspaces_->GetCell(i).GetStatus() == CellStatus::COVERED)
    {
      covered_count++;
    }
  }

  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    subspaces_->GetCell(cell_ind).ClearViewPointIndices();
  }
  for (const auto& viewpoint_ind : viewpoint_manager->candidate_indices_)
  {
    geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
    Eigen::Vector3i sub =
        subspaces_->Pos2Sub(Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));
    if (subspaces_->InRange(sub))
    {
      int cell_ind = subspaces_->Sub2Ind(sub);
      AddViewPointToCell(cell_ind, viewpoint_ind);
      viewpoint_manager->SetViewPointCellInd(viewpoint_ind, cell_ind);
    }
    else
    {
      ROS_ERROR_STREAM("subspace sub out of bound: " << sub.transpose());
    }
  }

  for (const auto& cell_ind : neighbor_cell_indices_)
  {
    if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED_BY_OTHERS)
    {
      continue;
    }
    int candidate_count = 0;
    int selected_viewpoint_count = 0;
    int above_big_threshold_count = 0;
    int above_small_threshold_count = 0;
    int above_frontier_threshold_count = 0;
    int highest_score_viewpoint_ind = -1;
    int highest_score = -1;
    for (const auto& viewpoint_ind : subspaces_->GetCell(cell_ind).GetViewPointIndices())
    {
      MY_ASSERT(viewpoint_manager->IsViewPointCandidate(viewpoint_ind));
      candidate_count++;
      if (viewpoint_manager->ViewPointSelected(viewpoint_ind))
      {
        selected_viewpoint_count++;
      }
      if (viewpoint_manager->ViewPointVisited(viewpoint_ind))
      {
        continue;
      }
      int score = viewpoint_manager->GetViewPointCoveredPointNum(viewpoint_ind);
      int frontier_score = viewpoint_manager->GetViewPointCoveredFrontierPointNum(viewpoint_ind);
      if (score > highest_score)
      {
        highest_score = score;
        highest_score_viewpoint_ind = viewpoint_ind;
      }
      if (score > kMinAddPointNumSmall)
      {
        above_small_threshold_count++;
      }
      if (score > kMinAddPointNumBig)
      {
        above_big_threshold_count++;
      }
      if (frontier_score > kMinAddFrontierPointNum)
      {
        above_frontier_threshold_count++;
      }
    }
    // if (highest_score_viewpoint_ind != -1)
    // {
    //   geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(highest_score_viewpoint_ind);
    //   subspaces_->GetCell(cell_ind).SetViewPointPosition(
    //       Eigen::Vector3d(viewpoint_position.x, viewpoint_position.y, viewpoint_position.z));
    // }
    // Exploring to Covered
    if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING &&
        above_frontier_threshold_count < kCellExploringToCoveredThr &&
        above_small_threshold_count < kCellExploringToCoveredThr && selected_viewpoint_count == 0 &&
        candidate_count > 0)
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
    }
    // Covered to Exploring
    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::COVERED &&
             (above_big_threshold_count >= kCellCoveredToExploringThr ||
              above_frontier_threshold_count >= kCellCoveredToExploringThr))
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
      // almost_covered_cell_indices_.erase(
      //     std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
      //     almost_covered_cell_indices_.end());
      // std::cout << "1 adding " << cell_ind << std::endl;
      almost_covered_cell_indices_.push_back(cell_ind);
    }
    // Exploring to Almost covered
    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && selected_viewpoint_count == 0 &&
             candidate_count > 0)
    // else if ((subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING ||
    //           subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::UNSEEN) &&
    //          selected_viewpoint_count == 0 && candidate_count > 0)
    {
      // std::cout << "2 adding " << cell_ind << std::endl;
      almost_covered_cell_indices_.push_back(cell_ind);
    }
    // Almost covered to Exploring
    else if (subspaces_->GetCell(cell_ind).GetStatus() != CellStatus::COVERED && selected_viewpoint_count > 0)
    {
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::EXPLORING);
      // std::cout << "removing " << cell_ind << std::endl;
      almost_covered_cell_indices_.erase(
          std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
          almost_covered_cell_indices_.end());
    }
    else if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && candidate_count == 0)
    {
      // First visit
      if (subspaces_->GetCell(cell_ind).GetVisitCount() == 1 &&
          subspaces_->GetCell(cell_ind).GetGraphNodeIndices().empty())
      {
        subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
      }
      else
      {
        geometry_msgs::Point cell_position = subspaces_->GetCell(cell_ind).GetPosition();
        double xy_dist_to_robot =
            misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(cell_position, robot_position_);
        double z_dist_to_robot = std::abs(cell_position.z - robot_position_.z);
        // if (std::abs(robot_position_.x - cell_position.x) < kCellSize * 0.8 &&
        //     std::abs(robot_position_.y - cell_position.y) < kCellSize * 0.8 && z_dist_to_robot < kCellHeight * 0.8)
        if (xy_dist_to_robot < kCellSize && z_dist_to_robot < kCellHeight * 0.8)
        {
          subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
        }
      }
    }

    if (subspaces_->GetCell(cell_ind).GetStatus() == CellStatus::EXPLORING && candidate_count > 0)
    {
      subspaces_->GetCell(cell_ind).SetRobotPosition(robot_position_);
      subspaces_->GetCell(cell_ind).SetKeyposeID(cur_keypose_id_);
    }
  }
  // std::cout << "before almost covered_cell_indices: " << std::endl;
  for (const auto& cell_ind : almost_covered_cell_indices_)
  {
    // std::cout << cell_ind << " ";
    if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), cell_ind) ==
        neighbor_cell_indices_.end())
    {
      // std::cout << "removing " << cell_ind << std::endl;
      subspaces_->GetCell(cell_ind).SetStatus(CellStatus::COVERED);
      almost_covered_cell_indices_.erase(
          std::remove(almost_covered_cell_indices_.begin(), almost_covered_cell_indices_.end(), cell_ind),
          almost_covered_cell_indices_.end());
    }
  }
  // std::cout << std::endl;
  // std::cout << "afler almost covered_cell_indices: " << std::endl;
  // for (const auto& cell_ind : almost_covered_cell_indices_)
  // {
  //   std::cout << cell_ind << " ";
  // }

  // std::cout << std::endl;
}

exploration_path_ns::ExplorationPath GridWorld::SolveGlobalTSP(
    const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
    std::vector<int>& ordered_cell_indices, const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  // Determine the node on keypose graph associated with the robot
  double min_dist_to_robot = DBL_MAX;
  geometry_msgs::Point global_path_robot_position = robot_position_;
  Eigen::Vector3d eigen_robot_position(robot_position_.x, robot_position_.y, robot_position_.z);
  // Get nearest connected node
  int closest_node_ind = 0;
  double closest_node_dist = DBL_MAX;
  keypose_graph->GetClosestConnectedNodeIndAndDistance(robot_position_, closest_node_ind, closest_node_dist);
  if (closest_node_dist < kCellSize / 2 && closest_node_ind >= 0 && closest_node_ind < keypose_graph->GetNodeNum())
  {
    global_path_robot_position = keypose_graph->GetNodePosition(closest_node_ind);
  }
  else if (cur_keypose_graph_node_ind_ >= 0 && cur_keypose_graph_node_ind_ < keypose_graph->GetNodeNum())
  {
    ROS_WARN("GridWorld::SolveGlobalTSP: using nearest keypose node for robot position");
    global_path_robot_position = keypose_graph->GetNodePosition(cur_keypose_graph_node_ind_);
  }
  else
  {
    ROS_WARN("GridWorld::SolveGlobalTSP: using neighbor cell roadmap connection points for robot position");
    for (int i = 0; i < neighbor_cell_indices_.size(); i++)
    {
      int cell_ind = neighbor_cell_indices_[i];
      if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
      {
        Eigen::Vector3d roadmap_connection_point = subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint();
        if (viewpoint_manager->InLocalPlanningHorizon(roadmap_connection_point))
        {
          double dist_to_robot = (roadmap_connection_point - eigen_robot_position).norm();
          if (dist_to_robot < min_dist_to_robot)
          {
            min_dist_to_robot = dist_to_robot;
            global_path_robot_position.x = roadmap_connection_point.x();
            global_path_robot_position.y = roadmap_connection_point.y();
            global_path_robot_position.z = roadmap_connection_point.z();
          }
        }
      }
    }
  }

  exploration_path_ns::ExplorationPath global_path;
  std::vector<geometry_msgs::Point> exploring_cell_positions;
  std::vector<int> exploring_cell_indices;
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING)
    {
      if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), i) == neighbor_cell_indices_.end() ||
          (subspaces_->GetCell(i).GetViewPointIndices().empty() && subspaces_->GetCell(i).GetVisitCount() > 1))
      {
        if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
        {
          // Use straight line connection
          exploring_cell_positions.push_back(GetCellPosition(i));
          exploring_cell_indices.push_back(i);
        }
        else
        {
          Eigen::Vector3d connection_point = subspaces_->GetCell(i).GetRoadmapConnectionPoint();
          geometry_msgs::Point connection_point_geo;
          connection_point_geo.x = connection_point.x();
          connection_point_geo.y = connection_point.y();
          connection_point_geo.z = connection_point.z();

          bool reachable = false;
          if (keypose_graph->IsPositionReachable(connection_point_geo))
          {
            reachable = true;
          }
          else
          {
            // Check all the keypose graph nodes within this cell to see if there are any connected nodes
            double min_dist = DBL_MAX;
            double min_dist_node_ind = -1;
            for (const auto& node_ind : subspaces_->GetCell(i).GetGraphNodeIndices())
            {
              geometry_msgs::Point node_position = keypose_graph->GetNodePosition(node_ind);
              double dist = misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
                  node_position, connection_point_geo);
              if (dist < min_dist)
              {
                min_dist = dist;
                min_dist_node_ind = node_ind;
              }
            }
            if (min_dist_node_ind >= 0 && min_dist_node_ind < keypose_graph->GetNodeNum())
            {
              reachable = true;
              connection_point_geo = keypose_graph->GetNodePosition(min_dist_node_ind);
            }
          }
          if (reachable)
          {
            exploring_cell_positions.push_back(connection_point_geo);
            exploring_cell_indices.push_back(i);
          }
        }
      }
    }
  }

  // std::cout << "exploring cell indices num: " << exploring_cell_indices.size() << std::endl;

  if (exploring_cell_indices.empty())
  {
    return_home_ = true;

    geometry_msgs::Point home_position;
    int home_cell_ind = GetCellInd(0, 0, 0);
    if (subspaces_->InRange(home_cell_ind))
    {
      Eigen::Vector3d home_eigen_position = subspaces_->GetCell(home_cell_ind).GetRoadmapConnectionPoint();
      home_position.x = home_eigen_position.x();
      home_position.y = home_eigen_position.y();
      home_position.z = home_eigen_position.z();
    }

    nav_msgs::Path return_home_path;
    if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
    {
      geometry_msgs::PoseStamped robot_pose;
      robot_pose.pose.position = robot_position_;

      geometry_msgs::PoseStamped home_pose;
      home_pose.pose.position = home_position;
      return_home_path.poses.push_back(robot_pose);
      return_home_path.poses.push_back(home_pose);
    }
    else
    {
      // home_position = keypose_graph->GetKeyposePosition(0);
      keypose_graph->GetShortestPath(global_path_robot_position, home_position, true, return_home_path, false);
      if (return_home_path.poses.size() >= 2)
      {
        global_path.FromPath(return_home_path);
        for (int i = 0; i < global_path.nodes_.size() - 1; i++)
        {
          global_path.nodes_[i].type_ = exploration_path_ns::NodeType::GLOBAL_KEYPOSE;
        }
        global_path.nodes_.back().type_ = exploration_path_ns::NodeType::HOME;
        // Make it a loop
        for (int i = global_path.nodes_.size() - 2; i > 0; i--)
        {
          global_path.Append(global_path.nodes_[i]);
        }
      }
      else
      {
        // ROS_ERROR("Cannot find path home");
        // TODO: find a path
      }
    }
    return global_path;
  }

  return_home_ = false;

  // Put the current robot position in the end
  exploring_cell_positions.push_back(global_path_robot_position);
  exploring_cell_indices.push_back(-1);

  // Construct the distance matrix
  std::vector<std::vector<int>> distance_matrix(exploring_cell_positions.size(),
                                                std::vector<int>(exploring_cell_positions.size(), 0));
  for (int i = 0; i < exploring_cell_positions.size(); i++)
  {
    for (int j = 0; j < i; j++)
    {
      if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
      {
        // Use straight line connection
        distance_matrix[i][j] =
            static_cast<int>(10 * misc_utils_ns::PointXYZDist<geometry_msgs::Point, geometry_msgs::Point>(
                                      exploring_cell_positions[i], exploring_cell_positions[j]));
      }
      else
      {
        // Use keypose graph
        nav_msgs::Path path_tmp;
        distance_matrix[i][j] =
            static_cast<int>(10 * keypose_graph->GetShortestPath(exploring_cell_positions[i],
                                                                 exploring_cell_positions[j], false, path_tmp, false));
      }
    }
  }

  for (int i = 0; i < exploring_cell_positions.size(); i++)
  {
    for (int j = i + 1; j < exploring_cell_positions.size(); j++)
    {
      distance_matrix[i][j] = distance_matrix[j][i];
    }
  }

  // // Print distance matrix
  // std::cout << "#####Printing distance matrix#####" << std::endl;
  // std::cout << "\t";
  // for (int i = 0; i < distance_matrix.size(); i++)
  // {
  //   std::cout << exploring_cell_indices[i] << "\t";
  // }
  // std::cout << std::endl;

  // for (int i = 0; i < distance_matrix.size(); i++)
  // {
  //   std::cout << exploring_cell_indices[i] << "\t";
  //   for (int j = 0; j < distance_matrix[i].size(); j++)
  //   {
  //     std::cout << distance_matrix[i][j] << "\t";
  //   }
  //   std::cout << std::endl;
  // }
  // std::cout << "#####Finishing printing distance matrix#####" << std::endl;
  tsp_solver_ns::DataModel data_model;
  data_model.distance_matrix = distance_matrix;
  data_model.depot = exploring_cell_positions.size() - 1;

  tsp_solver_ns::TSPSolver tsp_solver(data_model);
  tsp_solver.Solve();
  std::vector<int> node_index;
  tsp_solver.getSolutionNodeIndex(node_index, false);

  ordered_cell_indices.clear();

  if (!node_index.empty())
  {
    node_index.push_back(node_index[0]);
  }

  if (!use_keypose_graph_ || keypose_graph == nullptr || keypose_graph->GetNodeNum() == 0)
  {
    for (int i = 0; i < node_index.size(); i++)
    {
      int cell_ind = node_index[i];
      geometry_msgs::PoseStamped pose;
      pose.pose.position = exploring_cell_positions[cell_ind];
      exploration_path_ns::Node node(exploring_cell_positions[cell_ind],
                                     exploration_path_ns::NodeType::GLOBAL_VIEWPOINT);
      global_path.Append(node);
      ordered_cell_indices.push_back(exploring_cell_indices[cell_ind]);
    }
  }
  else
  {
    geometry_msgs::Point cur_position;
    geometry_msgs::Point next_position;
    int cur_keypose_id;
    int next_keypose_id;
    int cur_ind;
    int next_ind;

    for (int i = 0; i < node_index.size() - 1; i++)
    {
      cur_ind = node_index[i];
      next_ind = node_index[i + 1];
      cur_position = exploring_cell_positions[cur_ind];
      next_position = exploring_cell_positions[next_ind];

      nav_msgs::Path keypose_path;
      keypose_graph->GetShortestPath(cur_position, next_position, true, keypose_path, false);

      exploration_path_ns::Node node(Eigen::Vector3d(cur_position.x, cur_position.y, cur_position.z));
      if (i == 0)
      {
        node.type_ = exploration_path_ns::NodeType::ROBOT;
      }
      else
      {
        node.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
      }
      global_path.Append(node);
      if (keypose_path.poses.size() >= 2)
      {
        for (int j = 1; j < keypose_path.poses.size() - 1; j++)
        {
          geometry_msgs::Point node_position;
          node_position = keypose_path.poses[j].pose.position;
          exploration_path_ns::Node keypose_node(node_position, exploration_path_ns::NodeType::GLOBAL_VIA_POINT);
          keypose_node.keypose_graph_node_ind_ = static_cast<int>(keypose_path.poses[i].pose.orientation.x);
          global_path.Append(keypose_node);
        }
      }
      ordered_cell_indices.push_back(exploring_cell_indices[cur_ind]);
    }
    // // Append the last node
    // exploration_path_ns::Node node(Eigen::Vector3d(next_position.x, next_position.y, next_position.z));
    // if (next_keypose_id != -1)
    // {
    //   node.type_ = exploration_path_ns::NodeType::GLOBAL_VIEWPOINT;
    //   if (keypose_to_cell_map.find(next_keypose_id) != keypose_to_cell_map.end())
    //   {
    //     node.global_grid_indices_ = keypose_to_cell_map[next_keypose_id];
    //   }
    // }
    // else
    // {
    //   node.type_ = exploration_path_ns::NodeType::ROBOT;
    // }
    // global_path.Append(node);

    // ordered_cell_indices.push_back(keypose_id_list[node_index.back()]);

    // std::cout << "global exploration path node num: " << global_path.nodes_.size() << std::endl;
  }

  // for (int i = 0; i < node_index.size(); i++)
  // {
  //   int cell_ind = node_index[i];
  //   geometry_msgs::PoseStamped pose;
  //   pose.pose.position = exploring_cell_positions[cell_ind];
  //   global_tsp_path.poses.push_back(pose);
  //   if (use_keypose_graph_)
  //   {
  //     ordered_cell_indices.push_back(keypose_id_list[cell_ind]);
  //   }
  //   else
  //   {
  //     ordered_cell_indices.push_back(exploring_cell_indices[cell_ind]);
  //   }
  // }

  // std::cout << "path order: ";
  // for (int i = 0; i < ordered_cell_indices.size(); i++)
  // {
  //   std::cout << ordered_cell_indices[i] << " -> ";
  // }
  // std::cout << std::endl;

  return global_path;
}

int GridWorld::GetCellKeyposeID(int cell_ind)
{
  MY_ASSERT(subspaces_->InRange(cell_ind));
  return subspaces_->GetCell(cell_ind).GetKeyposeID();
}

void GridWorld::GetCellViewPointPositions(std::vector<Eigen::Vector3d>& viewpoint_positions)
{
  viewpoint_positions.clear();
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() != grid_world_ns::CellStatus::EXPLORING)
    {
      continue;
    }
    if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), i) == neighbor_cell_indices_.end())
    {
      viewpoint_positions.push_back(subspaces_->GetCell(i).GetViewPointPosition());
    }
  }
}

void GridWorld::AddPathsToKeyposeGraph(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  // Add a path directly to the keypose graph
  // The last node should be a keypose node
  for (int i = 0; i < subspaces_->GetCellNumber(); i++)
  {
    if (subspaces_->GetCell(i).GetStatus() == CellStatus::EXPLORING &&
        !subspaces_->GetCell(i).IsPathAddedToKeyposeGraph())
    {
      int cell_viewpoint_count = subspaces_->GetCell(i).GetViewPointIndices().size();
      if (std::find(neighbor_cell_indices_.begin(), neighbor_cell_indices_.end(), i) == neighbor_cell_indices_.end() ||
          cell_viewpoint_count == 0)
      {
        keypose_graph->AddToKeyposePath(subspaces_->GetCell(i).GetPathToKeyposeGraph());
        subspaces_->GetCell(i).SetPathAddedToKeyposeGraph(true);
      }
    }
  }
}

void GridWorld::AddPathsInBetweenCells(const std::shared_ptr<viewpoint_manager_ns::ViewPointManager>& viewpoint_manager,
                                       const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph)
{
  // Determine the connection point in each cell
  for (int i = 0; i < neighbor_cell_indices_.size(); i++)
  {
    int cell_ind = neighbor_cell_indices_[i];
    if (subspaces_->GetCell(cell_ind).IsRoadmapConnectionPointSet())
    {
      if (viewpoint_manager->InLocalPlanningHorizon(subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint()) &&
          !viewpoint_manager->InCollision(subspaces_->GetCell(cell_ind).GetRoadmapConnectionPoint()))
      {
        continue;
      }
      else
      {
        subspaces_->GetCell(cell_ind).ClearConnectedCellIndices();
      }
    }

    std::vector<int> candidate_viewpoint_indices = subspaces_->GetCell(cell_ind).GetViewPointIndices();
    if (!candidate_viewpoint_indices.empty())
    {
      double min_dist = DBL_MAX;
      double min_dist_viewpoint_ind = candidate_viewpoint_indices.front();
      for (const auto& viewpoint_ind : candidate_viewpoint_indices)
      {
        geometry_msgs::Point viewpoint_position = viewpoint_manager->GetViewPointPosition(viewpoint_ind);
        double dist_to_cell_center = misc_utils_ns::PointXYDist<geometry_msgs::Point, geometry_msgs::Point>(
            viewpoint_position, subspaces_->GetCell(cell_ind).GetPosition());
        if (dist_to_cell_center < min_dist)
        {
          min_dist = dist_to_cell_center;
          min_dist_viewpoint_ind = viewpoint_ind;
        }
      }
      geometry_msgs::Point min_dist_viewpoint_position =
          viewpoint_manager->GetViewPointPosition(min_dist_viewpoint_ind);
      subspaces_->GetCell(cell_ind).SetRoadmapConnectionPoint(
          Eigen::Vector3d(min_dist_viewpoint_position.x, min_dist_viewpoint_position.y, min_dist_viewpoint_position.z));
      subspaces_->GetCell(cell_ind).SetRoadmapConnectionPointSet(true);
    }
  }

  for (int i = 0; i < neighbor_cell_indices_.size(); i++)
  {
    int from_cell_ind = neighbor_cell_indices_[i];
    int viewpoint_num = subspaces_->GetCell(from_cell_ind).GetViewPointIndices().size();
    if (viewpoint_num == 0)
    {
      continue;
    }
    std::vector<int> from_cell_connected_cell_indices = subspaces_->GetCell(from_cell_ind).GetConnectedCellIndices();
    Eigen::Vector3d from_cell_roadmap_connection_position =
        subspaces_->GetCell(from_cell_ind).GetRoadmapConnectionPoint();
    if (!viewpoint_manager->InLocalPlanningHorizon(from_cell_roadmap_connection_position))
    {
      continue;
    }
    // Eigen::Vector3i from_cell_sub = ind2sub(from_cell_ind);
    Eigen::Vector3i from_cell_sub = subspaces_->Ind2Sub(from_cell_ind);
    std::vector<int> nearby_cell_indices;
    for (int x = -1; x <= 1; x++)
    {
      for (int y = -1; y <= 1; y++)
      {
        for (int z = -1; z <= 1; z++)
        {
          if (std::abs(x) + std::abs(y) + std::abs(z) == 1)
          {
            Eigen::Vector3i neighbor_sub = from_cell_sub + Eigen::Vector3i(x, y, z);
            // if (SubInBound(neighbor_sub))
            if (subspaces_->InRange(neighbor_sub))
            {
              // int neighbor_ind = sub2ind(neighbor_sub);
              int neighbor_ind = subspaces_->Sub2Ind(neighbor_sub);
              nearby_cell_indices.push_back(neighbor_ind);
            }
          }
        }
      }
    }

    for (int j = 0; j < nearby_cell_indices.size(); j++)
    {
      int to_cell_ind = nearby_cell_indices[j];
      // Just for debug
      if (!AreNeighbors(from_cell_ind, to_cell_ind))
      {
        ROS_ERROR_STREAM("Cell " << from_cell_ind << " and " << to_cell_ind << " are not neighbors");
      }
      if (subspaces_->GetCell(to_cell_ind).GetViewPointIndices().empty())
      {
        continue;
      }
      std::vector<int> to_cell_connected_cell_indices = subspaces_->GetCell(to_cell_ind).GetConnectedCellIndices();
      Eigen::Vector3d to_cell_roadmap_connection_position =
          subspaces_->GetCell(to_cell_ind).GetRoadmapConnectionPoint();
      if (!viewpoint_manager->InLocalPlanningHorizon(to_cell_roadmap_connection_position))
      {
        continue;
      }

      // TODO: change to: if there is already a direct keypose graph connection then continue
      bool connected_in_keypose_graph = HasDirectKeyposeGraphConnection(
          keypose_graph, from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);

      bool forward_connected =
          std::find(from_cell_connected_cell_indices.begin(), from_cell_connected_cell_indices.end(), to_cell_ind) !=
          from_cell_connected_cell_indices.end();
      bool backward_connected = std::find(to_cell_connected_cell_indices.begin(), to_cell_connected_cell_indices.end(),
                                          from_cell_ind) != to_cell_connected_cell_indices.end();

      if (connected_in_keypose_graph)
      {
        continue;
      }

      nav_msgs::Path path_in_between = viewpoint_manager->GetViewPointShortestPath(
          from_cell_roadmap_connection_position, to_cell_roadmap_connection_position);

      if (PathValid(path_in_between, from_cell_ind, to_cell_ind))
      {
        path_in_between = misc_utils_ns::SimplifyPath(path_in_between);
        for (auto& pose : path_in_between.poses)
        {
          pose.pose.orientation.w = -1;
        }
        // Add the path
        // std::cout << "Adding path between " << from_cell_ind << " " << to_cell_ind << std::endl;
        // to_connect_cell_paths_.push_back(path_in_between);
        keypose_graph->AddPath(path_in_between);
        bool connected = HasDirectKeyposeGraphConnection(keypose_graph, from_cell_roadmap_connection_position,
                                                         to_cell_roadmap_connection_position);
        if (!connected)
        {
          // Reset both cells' roadmap connection points
          // std::cout << "Resetting both cells connection points" << std::endl;
          subspaces_->GetCell(from_cell_ind).SetRoadmapConnectionPointSet(false);
          subspaces_->GetCell(to_cell_ind).SetRoadmapConnectionPointSet(false);
          subspaces_->GetCell(from_cell_ind).ClearConnectedCellIndices();
          subspaces_->GetCell(to_cell_ind).ClearConnectedCellIndices();
          continue;
        }
        else
        {
          subspaces_->GetCell(from_cell_ind).AddConnectedCell(to_cell_ind);
          subspaces_->GetCell(to_cell_ind).AddConnectedCell(from_cell_ind);
        }
      }
    }
  }
}

bool GridWorld::PathValid(const nav_msgs::Path& path, int from_cell_ind, int to_cell_ind)
{
  if (path.poses.size() >= 2)
  {
    for (const auto& pose : path.poses)
    {
      int cell_ind = GetCellInd(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
      if (cell_ind != from_cell_ind && cell_ind != to_cell_ind)
      {
        return false;
      }
    }
    return true;
  }
  else
  {
    return false;
  }
}

bool GridWorld::HasDirectKeyposeGraphConnection(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                                const Eigen::Vector3d& start_position,
                                                const Eigen::Vector3d& goal_position)
{
  if (!keypose_graph->HasNode(start_position) || !keypose_graph->HasNode(goal_position))
  {
    return false;
  }

  // Search a path connecting start_position and goal_position with a max path length constraint
  geometry_msgs::Point geo_start_position;
  geo_start_position.x = start_position.x();
  geo_start_position.y = start_position.y();
  geo_start_position.z = start_position.z();

  geometry_msgs::Point geo_goal_position;
  geo_goal_position.x = goal_position.x();
  geo_goal_position.y = goal_position.y();
  geo_goal_position.z = goal_position.z();

  double max_path_length = kCellSize * 2;
  nav_msgs::Path path;
  bool found_path =
      keypose_graph->GetShortestPathWithMaxLength(geo_start_position, geo_goal_position, max_path_length, false, path);
  return found_path;
}

}  // namespace grid_world_ns