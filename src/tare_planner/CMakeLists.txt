cmake_minimum_required(VERSION 3.5)
project(tare_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Eigen3 REQUIRED)

 include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include
  or-tools/include
  ${EIGEN3_INCLUDE_DIR}
  /opt/ros/humble/include
  # /usr/local/include
  # /usr/include
)

get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

# ## Specify additional locations for library files
# link_directories(
#   /usr/local/lib # Location when using 'make system_install'
#   /usr/lib       # More usual location (e.g. when installing using a package)
#   or-tools/lib
# )

add_library(lidar_model src/lidar_model/lidar_model.cpp)
target_link_libraries(lidar_model rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2 visualization_msgs::visualization_msgs pcl_conversions::pcl_conversions)
# target_link_libraries(lidar_model ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(graph src/graph/graph.cpp)
target_link_libraries(graph rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2 pcl_conversions::pcl_conversions visualization_msgs::visualization_msgs)
# target_link_libraries(graph ${catkin_LIBRARIES})

add_library(misc_utils src/utils/misc_utils.cpp)
target_link_libraries(misc_utils rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2 pcl_conversions::pcl_conversions visualization_msgs::visualization_msgs)
# target_link_libraries(misc_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(pointcloud_utils src/utils/pointcloud_utils.cpp)
target_link_libraries(pointcloud_utils rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2 pcl_conversions::pcl_conversions visualization_msgs::visualization_msgs)
# target_link_libraries(pointcloud_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(tsp_solver src/tsp_solver/tsp_solver.cpp)
target_link_libraries(tsp_solver rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2)
target_link_libraries(tsp_solver ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/or-tools/lib/libortools.so)

add_library(viewpoint src/viewpoint/viewpoint.cpp)
target_link_libraries(viewpoint rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2)
target_link_libraries(viewpoint ${catkin_LIBRARIES} lidar_model)

add_library(rolling_grid src/rolling_grid/rolling_grid.cpp)
target_link_libraries(rolling_grid rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2 pcl_conversions::pcl_conversions visualization_msgs::visualization_msgs)
target_link_libraries(rolling_grid ${catkin_LIBRARIES} misc_utils pointcloud_utils)

add_library(viewpoint_manager src/viewpoint_manager/viewpoint_manager.cpp)
target_link_libraries(viewpoint_manager rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2)
target_link_libraries(viewpoint_manager ${catkin_LIBRARIES} rolling_grid viewpoint grid_world)

add_library(local_coverage_planner src/local_coverage_planner/local_coverage_planner.cpp)
target_link_libraries(local_coverage_planner rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2)
target_link_libraries(local_coverage_planner ${catkin_LIBRARIES} viewpoint_manager)

add_library(grid_world src/grid_world/grid_world.cpp)
target_link_libraries(grid_world rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2)
target_link_libraries(grid_world ${catkin_LIBRARIES} viewpoint_manager tsp_solver keypose_graph exploration_path)

add_library(pointcloud_manager src/pointcloud_manager/pointcloud_manager.cpp)
target_link_libraries(pointcloud_manager rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2 pcl_conversions::pcl_conversions visualization_msgs::visualization_msgs)
target_link_libraries(pointcloud_manager ${catkin_LIBRARIES})

add_library(keypose_graph src/keypose_graph/keypose_graph.cpp)
target_link_libraries(keypose_graph rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2)
target_link_libraries(keypose_graph ${catkin_LIBRARIES} ${PCL_LIBRARIES} viewpoint_manager)

add_library(rolling_occupancy_grid src/rolling_occupancy_grid/rolling_occupancy_grid.cpp)
target_link_libraries(rolling_occupancy_grid rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2 pcl_conversions::pcl_conversions visualization_msgs::visualization_msgs)
target_link_libraries(rolling_occupancy_grid ${catkin_LIBRARIES})

add_library(planning_env src/planning_env/planning_env.cpp)
target_link_libraries(planning_env rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2)
target_link_libraries(planning_env ${catkin_LIBRARIES} ${PCL_LIBRARIES} rolling_occupancy_grid)

add_library(exploration_path src/exploration_path/exploration_path.cpp)
target_link_libraries(exploration_path rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2 pcl_conversions::pcl_conversions visualization_msgs::visualization_msgs)
target_link_libraries(exploration_path ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(tare_visualizer src/tare_visualizer/tare_visualizer.cpp)
target_link_libraries(tare_visualizer rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2 pcl_conversions::pcl_conversions visualization_msgs::visualization_msgs)
target_link_libraries(tare_visualizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(sensor_coverage_planner_ground src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp)
target_link_libraries(sensor_coverage_planner_ground rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2)
target_link_libraries(sensor_coverage_planner_ground ${catkin_LIBRARIES} graph planning_env keypose_graph viewpoint_manager pointcloud_manager grid_world local_coverage_planner tare_visualizer)

add_executable(navigationBoundary src/navigation_boundary_publisher/navigationBoundary.cpp)
target_link_libraries(navigationBoundary rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2 pcl_conversions::pcl_conversions)
target_link_libraries(navigationBoundary ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(tare_planner_node src/tare_planner_node/tare_planner_node.cpp)
target_link_libraries(tare_planner_node rclcpp::rclcpp std_msgs::std_msgs nav_msgs::nav_msgs geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs tf2_ros::tf2_ros tf2_geometry_msgs::tf2_geometry_msgs tf2::tf2)
target_link_libraries(tare_planner_node ${catkin_LIBRARIES} sensor_coverage_planner_ground)

#############
## Install ##
#############

install(TARGETS
  navigationBoundary
  tare_planner_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/
        )
install(DIRECTORY rviz/
        DESTINATION share/${PROJECT_NAME}/
        )
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}
      )
install(DIRECTORY data/
        DESTINATION share/${PROJECT_NAME}
      )

ament_package()


