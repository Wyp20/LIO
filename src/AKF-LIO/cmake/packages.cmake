list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

# Suppress PCL warnings
set(PCL_ALLOW_OLD_COMPILER ON)
set(PCL_DISABLE_GPU_WARNINGS ON)

# glog
find_package(Glog REQUIRED)
include_directories(${Glog_INCLUDE_DIRS})


find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        nav_msgs
        sensor_msgs
        roscpp
        rospy
        std_msgs
        pcl_ros
        tf
        message_generation
        eigen_conversions
        cv_bridge
        )


add_message_files(
        FILES
        Pose6D.msg
)

generate_messages(
        DEPENDENCIES
        geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(OpenCV REQUIRED)

# Define the required variables for catkin_package
set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
set(PCL_INCLUDE_DIRS ${PCL_INCLUDE_DIRS})
set(OpenCV_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})

catkin_package(
        CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime cv_bridge
        DEPENDS EIGEN3 PCL OpenCV
        INCLUDE_DIRS
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${PYTHON_INCLUDE_DIRS}
        ${yaml-cpp_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        include
)
