cmake_minimum_required(VERSION 2.8.3)
project(pv_lio)
add_definitions(-DDISABLE_LIVOX)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -pthread")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

ADD_COMPILE_OPTIONS(-std=c++14 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set( CMAKE_CXX_FLAGS "-std=c++14 -O3" )

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -pthread -std=c++0x -std=c++14 -fexceptions")

# LIO eval: force single-thread map update
add_definitions(-DMP_PROC_NUM=1)
message(STATUS "LIO eval: MP_PROC_NUM=1 (single-thread)")

find_package(Boost REQUIRED COMPONENTS timer)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rosbag
  rospy
  std_msgs
  pcl_ros
  tf
  message_generation
  eigen_conversions
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
#find_package(GTSAM REQUIRED QUIET)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/../../include
  
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

#add_message_files(
#  FILES
#  Pose6D.msg
#)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)

add_executable(pv_lio_node src/voxelMapping.cpp src/preprocess.cpp)
target_link_libraries(pv_lio_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pv_lio_node PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(pandar_to_velodyne src/converter/pandar_to_velodyne.cpp)
target_link_libraries(pandar_to_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(pandar_to_velodyne PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(rs_to_velodyne src/converter/rs_to_velodyne.cpp)
target_link_libraries(rs_to_velodyne ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES})
target_include_directories(rs_to_velodyne PRIVATE ${PYTHON_INCLUDE_DIRS})
