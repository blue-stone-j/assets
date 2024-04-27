cmake_minimum_required(VERSION 3.0)
project(pcl_visualizer)

# 寻找PCL库
find_package(PCL 1.10.1)

# find_package(pcl_ros)

# 包含PCL头文件目录
include_directories(${PCL_INCLUDE_DIRS})

# 添加编译选项
add_definitions(${PCL_DEFINITIONS})

# 设置可执行文件
add_executable(pcl_visualizer pclv.cpp)

# 链接PCL库到你的可执行文件
target_link_libraries(pcl_visualizer ${PCL_LIBRARIES})
