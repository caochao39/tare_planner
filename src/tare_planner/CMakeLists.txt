cmake_minimum_required(VERSION 3.1)
project(tare_planner)

# add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  pcl_ros
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES coverage_planner
  CATKIN_DEPENDS nav_msgs roscpp rospy sensor_msgs std_msgs pcl_ros 
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  or-tools/include
  ${EIGEN3_INCLUDE_DIR}
  /opt/nea/topaz/include
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
  or-tools/lib
)

add_library(lidar_model src/lidar_model/lidar_model.cpp)
add_dependencies(lidar_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lidar_model ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(misc_utils src/utils/misc_utils.cpp)
add_dependencies(misc_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(misc_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(pointcloud_utils src/utils/pointcloud_utils.cpp)
add_dependencies(pointcloud_utils ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_utils ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(tsp_solver src/tsp_solver/tsp_solver.cpp)
add_dependencies(tsp_solver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tsp_solver ${catkin_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/or-tools/lib/libortools.so ${CMAKE_CURRENT_SOURCE_DIR}/or-tools/lib/libglog.so)

add_library(viewpoint src/viewpoint/viewpoint.cpp)
add_dependencies(viewpoint ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viewpoint ${catkin_LIBRARIES} lidar_model)

add_library(rollable_grid src/rollable_grid/rollable_grid.cpp)
add_dependencies(rollable_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(rollable_grid ${catkin_LIBRARIES} misc_utils pointcloud_utils)

add_library(viewpoint_manager src/viewpoint_manager/viewpoint_manager.cpp)
add_dependencies(viewpoint_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(viewpoint_manager ${catkin_LIBRARIES} rollable_grid viewpoint grid_world)

add_library(local_coverage_planner src/local_coverage_planner/local_coverage_planner.cpp)
add_dependencies(local_coverage_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(local_coverage_planner ${catkin_LIBRARIES} viewpoint_manager)

add_library(grid_world src/grid_world/grid_world.cpp)
add_dependencies(grid_world ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(grid_world ${catkin_LIBRARIES} viewpoint_manager tsp_solver keypose_graph exploration_path)

add_library(pointcloud_manager src/pointcloud_manager/pointcloud_manager.cpp)
add_dependencies(pointcloud_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloud_manager ${catkin_LIBRARIES})

add_library(keypose_graph src/keypose_graph/keypose_graph.cpp)
add_dependencies(keypose_graph ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(keypose_graph ${catkin_LIBRARIES} ${PCL_LIBRARIES} viewpoint_manager)

add_library(occupancy_grid src/occupancy_grid/occupancy_grid.cpp)
add_dependencies(occupancy_grid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(occupancy_grid ${catkin_LIBRARIES})

add_library(planning_env src/planning_env/planning_env.cpp)
add_dependencies(planning_env ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_env ${catkin_LIBRARIES} ${PCL_LIBRARIES} occupancy_grid)

add_library(exploration_path src/exploration_path/exploration_path.cpp)
add_dependencies(exploration_path ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(exploration_path ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(tare_visualizer src/tare_visualizer/tare_visualizer.cpp)
add_dependencies(tare_visualizer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tare_visualizer ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_library(sensor_coverage_planner_ground src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp)
add_dependencies(sensor_coverage_planner_ground ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sensor_coverage_planner_ground ${catkin_LIBRARIES} planning_env keypose_graph viewpoint_manager pointcloud_manager grid_world local_coverage_planner tare_visualizer)

add_executable(tare_planner_node src/tare_planner_node/tare_planner_node.cpp)
add_dependencies(tare_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
target_link_libraries(tare_planner_node ${catkin_LIBRARIES} sensor_coverage_planner_ground)

#############
## Install ##
#############

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
        )
install(DIRECTORY rviz/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
        )

#############
## Testing ##
#############


