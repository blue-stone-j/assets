cmake_minimum_required(VERSION 3.10)
project(convert_cloud)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag REQUIRED)

include(cmake/YAML.cmake)
include(cmake/glog.cmake)
include(cmake/PCL.cmake)

include_directories(
  include
  ${roscpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${rosbag_INCLUDE_DIRS}
)

add_executable(bag2pcd_node src/bag2pcd_node.cpp)
target_link_libraries(bag2pcd_node
  ${roscpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${rosbag_LIBRARIES}
  ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES} gflags)

add_executable(bag2bin_node src/bag2bin_node.cpp)
target_link_libraries(bag2bin_node
  ${roscpp_LIBRARIES} ${sensor_msgs_LIBRARIES} ${rosbag_LIBRARIES}
  ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${GLOG_LIBRARIES} gflags)

add_executable(bin2pcd_node src/bin2pcd_node.cpp)
target_link_libraries(bin2pcd_node ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${GLOG_LIBRARIES} gflags)

add_executable(pcd2bin_node src/pcd2bin_node.cpp)
target_link_libraries(pcd2bin_node ${YAML_CPP_LIBRARIES} ${PCL_LIBRARIES} ${GLOG_LIBRARIES} gflags)

add_executable(pcd_v src/pcd_v.cpp)
target_link_libraries(pcd_v ${YAML_CPP_LIBRARIES} ${GLOG_LIBRARIES} ${PCL_LIBRARIES} gflags)