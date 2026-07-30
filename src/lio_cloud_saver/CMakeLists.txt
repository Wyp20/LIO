cmake_minimum_required(VERSION 3.0.2)
project(lio_cloud_saver)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  pcl_conversions
)
find_package(PCL REQUIRED COMPONENTS common io)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(save_clouds_node src/save_clouds_node.cpp)
target_link_libraries(save_clouds_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS save_clouds_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
