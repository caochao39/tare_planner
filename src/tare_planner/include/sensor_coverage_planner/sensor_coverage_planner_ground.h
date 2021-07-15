/**
 * @file sensor_coverage_planner_ground.h
 * @author Chao Cao (ccao1@andrew.cmu.edu)
 * @brief Class that does the job of exploration
 * @version 0.1
 * @date 2020-06-03
 *
 * @copyright Copyright (c) 2021
 *
 */
#pragma once

#include <cmath>
#include <vector>

#include <Eigen/Core>
// ROS
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Pose.h>
// PCL
#include <pcl/PointIndices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
// Third parties
#include <utils/pointcloud_utils.h>
#include <utils/misc_utils.h>
// Components
#include "keypose_graph/keypose_graph.h"
#include "planning_env/planning_env.h"
#include "viewpoint_manager/viewpoint_manager.h"
#include "grid_world/grid_world.h"
#include "exploration_path/exploration_path.h"
#include "local_coverage_planner/local_coverage_planner.h"
#include "tare_visualizer/tare_visualizer.h"
#include "rolling_occupancy_grid/rolling_occupancy_grid.h"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

namespace sensor_coverage_planner_3d_ns
{
const std::string kWorldFrameID = "map";
typedef pcl::PointXYZRGBNormal PlannerCloudPointType;
typedef pcl::PointCloud<PlannerCloudPointType> PlannerCloudType;
typedef misc_utils_ns::Timer Timer;

struct PlannerParameters
{
  // String
  std::string sub_start_exploration_topic_;
  std::string sub_keypose_topic_;
  std::string sub_state_estimation_topic_;
  std::string sub_registered_scan_topic_;
  std::string sub_terrain_map_topic_;
  std::string sub_terrain_map_ext_topic_;
  std::string sub_coverage_boundary_topic_;
  std::string sub_viewpoint_boundary_topic_;
  std::string sub_nogo_boundary_topic_;

  std::string pub_exploration_finish_topic_;
  std::string pub_runtime_breakdown_topic_;
  std::string pub_runtime_topic_;
  std::string pub_waypoint_topic_;

  // Bool
  bool kAutoStart;
  bool kRushHome;
  bool kUseTerrainHeight;
  bool kCheckTerrainCollision;
  bool kExtendWayPoint;
  bool kUseLineOfSightLookAheadPoint;
  bool kNoExplorationReturnHome;

  // Double
  double kKeyposeCloudDwzFilterLeafSize;
  double kRushHomeDist;
  double kAtHomeDistThreshold;
  double kTerrainCollisionThreshold;
  double kLookAheadDistance;
  double kExtendWayPointDistance;

  bool ReadParameters(ros::NodeHandle& nh);
};

struct PlannerData
{
  // PCL clouds TODO: keypose cloud does not need to be PlannerCloudPointType
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<PlannerCloudPointType>> keypose_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZ>> registered_scan_stack_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> registered_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> large_terrain_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> terrain_ext_collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> viewpoint_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> grid_world_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> selected_viewpoint_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> exploring_cell_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> exploration_path_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> lookahead_point_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> keypose_graph_vis_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> viewpoint_in_collision_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> point_cloud_manager_neighbor_cloud_;
  std::unique_ptr<pointcloud_utils_ns::PCLCloud<pcl::PointXYZI>> reordered_global_subspace_cloud_;

  nav_msgs::Odometry keypose_;
  geometry_msgs::Point robot_position_;
  lidar_model_ns::LiDARModel robot_viewpoint_;
  exploration_path_ns::ExplorationPath exploration_path_;
  Eigen::Vector3d lookahead_point_;
  Eigen::Vector3d moving_direction_;
  double robot_yaw_;
  bool moving_forward_;
  std::vector<Eigen::Vector3d> visited_positions_;
  int cur_keypose_node_ind_;
  Eigen::Vector3d initial_position_;

  std::unique_ptr<keypose_graph_ns::KeyposeGraph> keypose_graph_;
  std::unique_ptr<planning_env_ns::PlanningEnv> planning_env_;
  std::shared_ptr<viewpoint_manager_ns::ViewPointManager> viewpoint_manager_;
  std::unique_ptr<local_coverage_planner_ns::LocalCoveragePlanner> local_coverage_planner_;
  std::unique_ptr<grid_world_ns::GridWorld> grid_world_;
  std::unique_ptr<tare_visualizer_ns::TAREVisualizer> visualizer_;

  std::unique_ptr<misc_utils_ns::Marker> keypose_graph_node_marker_;
  std::unique_ptr<misc_utils_ns::Marker> keypose_graph_edge_marker_;
  std::unique_ptr<misc_utils_ns::Marker> nogo_boundary_marker_;
  std::unique_ptr<misc_utils_ns::Marker> grid_world_marker_;

  void Initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
};

class SensorCoveragePlanner3D
{
public:
  explicit SensorCoveragePlanner3D(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
  bool initialize(ros::NodeHandle& nh, ros::NodeHandle& nh_p);
  void execute(const ros::TimerEvent&);
  ~SensorCoveragePlanner3D() = default;

private:
  bool keypose_cloud_update_;
  bool initialized_;
  bool lookahead_point_update_;
  bool relocation_;
  bool start_exploration_;
  bool exploration_finished_;
  bool near_home_;
  bool at_home_;
  bool stopped_;
  bool test_point_update_;
  bool viewpoint_ind_update_;
  bool step_;
  PlannerParameters pp_;
  PlannerData pd_;
  pointcloud_utils_ns::PointCloudDownsizer<pcl::PointXYZ> pointcloud_downsizer_;

  int update_representation_runtime_;
  int local_viewpoint_sampling_runtime_;
  int local_path_finding_runtime_;
  int global_planning_runtime_;
  int trajectory_optimization_runtime_;
  int overall_runtime_;
  int registered_cloud_count_;
  int keypose_count_;

  ros::Time start_time_;

  ros::Timer execution_timer_;

  // ROS subscribers
  ros::Subscriber exploration_start_sub_;
  ros::Subscriber state_estimation_sub_;
  ros::Subscriber registered_scan_sub_;
  ros::Subscriber terrain_map_sub_;
  ros::Subscriber terrain_map_ext_sub_;
  ros::Subscriber coverage_boundary_sub_;
  ros::Subscriber viewpoint_boundary_sub_;
  ros::Subscriber nogo_boundary_sub_;

  // ROS publishers
  ros::Publisher global_path_full_publisher_;
  ros::Publisher global_path_publisher_;
  ros::Publisher old_global_path_publisher_;
  ros::Publisher to_nearest_global_subspace_path_publisher_;
  ros::Publisher local_tsp_path_publisher_;
  ros::Publisher exploration_path_publisher_;
  ros::Publisher waypoint_pub_;
  ros::Publisher exploration_finish_pub_;
  ros::Publisher runtime_breakdown_pub_;
  ros::Publisher runtime_pub_;
  // Debug
  ros::Publisher pointcloud_manager_neighbor_cells_origin_pub_;

  // Callback functions
  void ExplorationStartCallback(const std_msgs::Bool::ConstPtr& start_msg);
  void StateEstimationCallback(const nav_msgs::Odometry::ConstPtr& state_estimation_msg);
  void RegisteredScanCallback(const sensor_msgs::PointCloud2ConstPtr& registered_cloud_msg);
  void TerrainMapCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_map_msg);
  void TerrainMapExtCallback(const sensor_msgs::PointCloud2ConstPtr& terrain_cloud_large_msg);
  void CoverageBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg);
  void ViewPointBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg);
  void NogoBoundaryCallback(const geometry_msgs::PolygonStampedConstPtr& polygon_msg);

  void SendInitialWaypoint();
  void UpdateKeyposeGraph();
  int UpdateViewPoints();
  void UpdateViewPointCoverage();
  void UpdateRobotViewPointCoverage();
  void UpdateCoveredAreas(int& uncovered_point_num, int& uncovered_frontier_point_num);
  void UpdateVisitedPositions();
  void UpdateGlobalRepresentation();
  void GlobalPlanning(std::vector<int>& global_cell_tsp_order, exploration_path_ns::ExplorationPath& global_path);
  void ReorderGlobalPath(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                         exploration_path_ns::ExplorationPath& global_path, std::vector<int>& global_path_node_indices);
  int GetNearestGlobalSubspace(const exploration_path_ns::ExplorationPath& global_path,
                               const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                               nav_msgs::Path& path_to_nearest_global_subspace);
  void GetShorestPathToNearestGlobalSubspace(const exploration_path_ns::ExplorationPath& global_path,
                                             const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                                             std::vector<int>& global_path_node_indices,
                                             nav_msgs::Path& path_to_nearest_subspace);
  bool ReorderGlobalSubspaceIndices(int nearest_global_subspace_index, std::vector<int>& global_subspace_indices);
  void ReconstructGlobalPath(const std::unique_ptr<keypose_graph_ns::KeyposeGraph>& keypose_graph,
                             const std::vector<int>& global_subspace_indices,
                             exploration_path_ns::ExplorationPath& global_path);
  void PublishGlobalPlanningVisualization(const exploration_path_ns::ExplorationPath& global_path,
                                          const exploration_path_ns::ExplorationPath& local_path);
  void LocalPlanning(int uncovered_point_num, int uncovered_frontier_point_num,
                     const exploration_path_ns::ExplorationPath& global_path,
                     exploration_path_ns::ExplorationPath& local_path);
  void PublishLocalPlanningVisualization(const exploration_path_ns::ExplorationPath& local_path);
  exploration_path_ns::ExplorationPath ConcatenateGlobalLocalPath(
      const exploration_path_ns::ExplorationPath& global_path, const exploration_path_ns::ExplorationPath& local_path);

  void PublishRuntime();
  double GetRobotToHomeDistance();
  void PublishExplorationState();
  void PublishWaypoint();
  bool GetLookAheadPoint(const exploration_path_ns::ExplorationPath& local_path,
                         const exploration_path_ns::ExplorationPath& global_path, Eigen::Vector3d& lookahead_point);

  void PrintExplorationStatus(std::string status, bool clear_last_line = true);
};

}  // namespace sensor_coverage_planner_3d_ns
