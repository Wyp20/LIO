# R-VoxelMap: Accurate Voxel Mapping with Recursive Plane Fitting for Online LiDAR Odometry

<div align="left">
<a href="https://arxiv.org/abs/2601.12377"><img alt="Paper" src="https://img.shields.io/badge/Paper-arXiv-%23B31B1B"/></a>
<a href="https://ieeexplore.ieee.org/document/11352826"><img alt="Paper" src="https://img.shields.io/badge/Paper-IEEE RAL-blue"/></a>
<a href="https://www.bilibili.com/video/BV1596MBSEWV"><img alt="Paper" src="https://img.shields.io/badge/Video-Bilibili-FB7299"/></a>
</div>

## Introduction
**R-VoxelMap** is a voxel mapping method that improves localization accuracy in online LiDAR odometry by using a geometry-driven recursive plane fitting strategy. Our code is based on [VoxelMap](https://github.com/hku-mars/VoxelMap) and primarily addresses the issues where VoxelMap and its variants typically fit and check planes using all points in a voxel, leading to parameter deviation due to outliers, over-segmentation of large planes, and incorrect merging across different physical planes. The main changes are as follows:

1. R-VoxelMap performs an outlier detect-and-reuse pipeline in each recursive iteration, effectively suppressing outlier influence, improving map accuracy, and reducing plane over-segmentation.

2. R-VoxelMap uses a point distribution-based plane validity check strategy, projecting and clustering point clouds on the RANSAC-fitted plane to prevent incorrect merging of different physical planes.


### Related paper
Related paper available on **[arxiv](https://arxiv.org/abs/2601.12377)** and **[IEEE RAL 2026](https://ieeexplore.ieee.org/document/11352826)**.


<div align="center">
    <img src="pictures/framework.png" width = 100%>
    <br>
    <font color=#a0a0a0 size=2>The framework of Lidar(-inertial) odometry based on R-VoxelMap.</font>
</div>
<div align="center">
    <img src="pictures/construction.png" width = 100%>
    <br>
    <font color=#a0a0a0 size=2>The construction steps of R-VoxelMap. (outlier detect-and-reuse pipeline)</font>
</div>

## 1. Dependencies
The required dependencies are same as VoxelMap. 
### 1.1. **PCL && Eigen**
PCL>= 1.8, Eigen>= 3.3.4 

### 1.2. **livox_ros_driver**
Follow [livox_ros_driver Installation](https://github.com/Livox-SDK/livox_ros_driver).

## 2. Build
Clone the repository and catkin_make:
```
    cd ~/$A_ROS_DIR$/src
    git clone https://github.com/NKU-MobFly-Robotics/R-VoxelMap.git
    cd ..
    catkin_make
    source devel/setup.bash
```
- Remember to source the livox_ros_driver before build (follow 1.2 **livox_ros_driver**).

## 3. Run
### 3.1 Run on rosbag
KITTI odometry dataset for example.
The KITTI rosbag we used can be downloaded from [Baidu Netdisk](https://pan.baidu.com/s/1w_TbMYSE7PTLqHWSJRBX0A?pwd=adjq).
```
    cd ~/$R_VOXEL_MAP_ROS_DIR$
    source devel/setup.bash
    roslaunch rvoxelmap kitti.launch
    rosbag play kitti_00.bag --delay 1
```

- Make sure the topics match the rostopics you are using before running.
- Make sure log file path is modified to a suitable path.
- It is **not recommended** to enable the visualization option for R-VoxelMap plane features, as it may affect efficiency. If visualization is needed, you can use rosservice `/rvoxelmap/publish_all` for one-time visualization.
- `--delay 1` is used to ensure rosbag's first frame data can be captured by the algorithm.

### 3.2 Run for test
We provide programs for testing the construction and update of R-VoxelMap. 

#### Test construction (using PCD file)

```
    cd ~/$R_VOXEL_MAP_ROS_DIR$
    source devel/setup.bash
    roslaunch rvoxelmap test.launch 
```
- Make sure `test.launch`'s `pcd_file` parameter points to your own PCD file.

#### Test update (using rosbag)
```
    cd ~/$R_VOXEL_MAP_ROS_DIR$
    source devel/setup.bash
    roslaunch rvoxelmap test_update.launch 
    rosbag play test.bag
```
- You need to prepare a test bag file containing point cloud data in PointCloud2 format.
- Make sure `test_update.launch` input point cloud topic remap to the correct topic.
- It is also **not recommended** to enable the visualization option for R-VoxelMap plane features, as it may affect efficiency. If visualization is needed, you can use rosservice `/publish_again` or `/publish_all` for one-time visualization.

## Acknowledgments
Our code is built on top of [VoxelMap](https://github.com/hku-mars/VoxelMap). We would like to acknowledge the contributions of the VoxelMap team for providing such a valuable framework.

