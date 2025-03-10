cmake_minimum_required(VERSION 3.0.2)
project(graph_slam)

add_definitions(-std=c++11 -O3)

set(G2O_INCLUDE_DIRS /usr/local/include)
set(G2O_CORE_LIBRARY /usr/local/lib/libg2o_core.so)
set(G2O_STUFF_LIBRARY /usr/local/lib/libg2o_stuff.so)
set(G2O_SOLVER_PCG /usr/local/lib/libg2o_solver_pcg.so)
set(G2O_SOLVER_CSPARSE /usr/local/lib/libg2o_solver_csparse.so)
set(G2O_SOLVER_CHOLMOD /usr/local/lib/libg2o_solver_cholmod.so)
set(G2O_TYPES_SLAM2D /usr/local/lib/libg2o_types_slam2d.so)
set(G2O_TYPES_SLAM2D_ADDONS /usr/local/lib/libg2o_types_slam2d_addons.so)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  pcl_ros
  roscpp
  sensor_msgs
  tf
  nodelet
  pluginlib
)

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES graph_slam
  CATKIN_DEPENDS geometry_msgs nav_msgs pcl_ros roscpp sensor_msgs tf 
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_library(graph_slam_nodelet
  src/slam_algorithm.cpp 
  src/robust_kernel_io.cpp 
  src/slam.cpp 
  src/nodelet.cpp
)
target_link_libraries(graph_slam_nodelet ${catkin_LIBRARIES})

add_executable(graph_slam_node
  src/slam_algorithm.cpp 
  src/robust_kernel_io.cpp 
  src/slam.cpp 
  src/main.cpp
)

target_link_libraries(graph_slam_node
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_PCG}
  ${G2O_SOLVER_CSPARSE}
  ${G2O_SOLVER_CHOLMOD}
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM2D_ADDONS}
)
add_dependencies(graph_slam_node ${catkin_EXPORTED_TARGETS})

install(TARGETS graph_slam_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
