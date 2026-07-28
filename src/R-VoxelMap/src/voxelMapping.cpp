#include "IMU_Processing.hpp"
#include "preprocess.h"
#include "voxel_map_util.hpp"
#include <Eigen/Core>
#include <common_lib.h>
#include <csignal>
#include <cv_bridge/cv_bridge.h>
#include <fstream>
#include <geometry_msgs/Vector3.h>
#include <image_transport/image_transport.h>
#ifndef DISABLE_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif
#include <math.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <opencv2/opencv.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include "offline_bag_feed.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <so3_math.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>
#include <thread>
#include <unistd.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <rvoxelmap/States.h>
#include <sys/resource.h>
#include <std_srvs/Empty.h> 

#define INIT_TIME (0.0)
#define CALIB_ANGLE_COV (0.01)
bool calib_laser = false;

// params for imu
bool imu_en = true;
std::vector<double> extrinT;
std::vector<double> extrinR;

// R-VoxelMap instance
VoxelMap voxelmap;

// params for publish function
bool pub_update_plane = false;
bool publish_point_cloud = false;
int pub_point_cloud_skip = 1;
bool pub_effect = false;
double intensity_min_thr = 0.0, intensity_max_thr = 1.0;

// record point usage
double mean_effect_points = 0;
double mean_ds_points = 0;
double mean_raw_points = 0;

// record time
double undistort_time_mean = 0;
double down_sample_time_mean = 0;
double calc_cov_time_mean = 0;
double scan_match_time_mean = 0;
double ekf_solve_time_mean = 0;
double map_update_time_mean = 0;

mutex mtx_buffer;
condition_variable sig_buffer;
Eigen::Vector3d last_odom(0, 0, 0);
Eigen::Matrix3d last_rot = Eigen::Matrix3d::Zero();
double trajectory_len = 0;

string map_file_path, lid_topic, imu_topic;
int scanIdx = 0;

int iterCount, feats_down_size, NUM_MAX_ITERATIONS, laserCloudValidNum,
    effct_feat_num, time_log_counter, publish_count = 0;

double first_lidar_time = 0;
double lidar_end_time = 0;
double res_mean_last = 0.05;
double total_distance = 0;
double gyr_cov_scale, acc_cov_scale;
double last_timestamp_lidar, last_timestamp_imu = -1.0;
double filter_size_corner_min, filter_size_surf_min, fov_deg;
double map_incremental_time, kdtree_search_time, total_time, scan_match_time,
    solve_time;
bool lidar_pushed, flg_reset, flg_exit = false;
bool dense_map_en = true;

deque<PointCloudXYZI::Ptr> lidar_buffer;
deque<double> time_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

// surf feature in map
PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr cube_points_add(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr map_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));       // 一帧可以找到有效匹配的原始点（lidar系）
PointCloudXYZI::Ptr laserCloudNoeffect(new PointCloudXYZI(100000, 1));
pcl::VoxelGrid<PointType> downSizeFilterSurf;
pcl::VoxelGrid<PointType> downSizeFilterMap;

V3D euler_cur;
V3D position_last(Zero3d);

// estimator inputs and output;
MeasureGroup Measures;
StatesGroup state;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

// my log
string tum_file_path, mem_file_path, time_file_path, time_aver_file_path, kitti_file_path;
bool save_tum_flag, save_mem_flag, save_time_flag, save_kitti_flag;
vector<double> time_vec_aver(6,0.0); // imup, downsample, ekf_update, map_update, other, total
vector<double> time_vec_tmp(6,0.0); // imup, downsample, ekf_update, map_update, other, total
int log_scan_num = 0;

// file stream
std::ofstream tum_odom_file;
std::ofstream kitti_odom_file;
std::ofstream mem_file;
std::ofstream time_file;
std::ofstream time_aver_file;

std::vector<ptpl> ptpl_list;
std::vector<Eigen::Vector3d> effect_points;
std::vector<Eigen::Vector3d> non_match_list;

shared_ptr<Preprocess> p_pre(new Preprocess());

// publisher
ros::Publisher voxel_map_all_plane_pub;
ros::Publisher voxel_map_all_plane_layer_color_pub;
ros::Publisher voxel_map_all_point_pub;
ros::Publisher voxel_map_grid_map_pub;

// params for publish function
int publish_max_voxel_layer = 0;
bool pub_all_plane_enable = false;
bool pub_all_plane_layer_color_enable = false;
bool pub_all_point_enable = false;
bool pub_grid_map_enable = false;

void SigHandle(int sig) {
  flg_exit = true;
  ROS_WARN("catch sig %d", sig);
  sig_buffer.notify_all();
}

const bool intensity_contrast(PointType &x, PointType &y) {
  return (x.intensity > y.intensity);
};

const bool var_contrast(pointWithCov &x, pointWithCov &y) {
  return (x.cov.diagonal().norm() < y.cov.diagonal().norm());
};

// project the lidar scan to world frame
// state is IMU pose in world; apply LiDAR->IMU extrinsic first
void pointBodyToWorld(PointType const *const pi, PointType *const po) {
  V3D p_lidar(pi->x, pi->y, pi->z);
  V3D p_body = Lidar_rot_to_IMU * p_lidar + Lidar_offset_to_IMU;
  V3D p_global(state.rot_end * p_body + state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

template <typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po) {
  V3D p_lidar(pi[0], pi[1], pi[2]);
  V3D p_body = Lidar_rot_to_IMU * p_lidar + Lidar_offset_to_IMU;
  V3D p_global(state.rot_end * p_body + state.pos_end);
  po[0] = p_global(0);
  po[1] = p_global(1);
  po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const *const pi, PointType *const po) {
  V3D p_lidar(pi->x, pi->y, pi->z);
  V3D p_body = Lidar_rot_to_IMU * p_lidar + Lidar_offset_to_IMU;
  V3D p_global(state.rot_end * p_body + state.pos_end);
  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
  po->curvature = pi->curvature;
  po->normal_x = pi->normal_x;
  po->normal_y = pi->normal_y;
  po->normal_z = pi->normal_z;
  float intensity = pi->intensity;
  intensity = intensity - floor(intensity);

  int reflection_map = intensity * 10000;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) {
  mtx_buffer.lock();
  // cout<<"got feature"<<endl;
  if (msg->header.stamp.toSec() < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(msg->header.stamp.toSec());
  last_timestamp_lidar = msg->header.stamp.toSec();

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

#ifndef DISABLE_LIVOX
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
  mtx_buffer.lock();
  // cout << "got feature" << endl;
  if (msg->header.stamp.toSec() < last_timestamp_lidar) {
    ROS_ERROR("lidar loop back, clear buffer");
    lidar_buffer.clear();
  }
  // ROS_INFO("get point cloud at time: %.6f", msg->header.stamp.toSec());
  PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
  p_pre->process(msg, ptr);
  lidar_buffer.push_back(ptr);
  time_buffer.push_back(msg->header.stamp.toSec());
  last_timestamp_lidar = msg->header.stamp.toSec();

  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

#endif

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) {
  publish_count++;
  sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

  double timestamp = msg->header.stamp.toSec();

  mtx_buffer.lock();

  if (timestamp < last_timestamp_imu) {
    ROS_ERROR("imu loop back, clear buffer");
    imu_buffer.clear();
    flg_reset = true;
  }

  last_timestamp_imu = timestamp;

  imu_buffer.push_back(msg);
  // cout << "got imu: " << timestamp << " imu size " << imu_buffer.size() << endl;
  mtx_buffer.unlock();
  sig_buffer.notify_all();
}

bool sync_packages(MeasureGroup &meas) {
  if (!imu_en) {
    if (!lidar_buffer.empty()) {
      // cout<<"meas.lidar->points.size(): "<<meas.lidar->points.size()<<endl;
      meas.lidar = lidar_buffer.front();
      meas.lidar_beg_time = time_buffer.front();
      lidar_end_time = meas.lidar_beg_time;
      time_buffer.pop_front();
      lidar_buffer.pop_front();
      return true;
    }

    return false;
  }

  if (lidar_buffer.empty() || imu_buffer.empty()) {
    return false;
  }

  /*** push a lidar scan ***/
  if (!lidar_pushed) {
    meas.lidar = lidar_buffer.front();
    if (meas.lidar->points.size() <= 1) {
      lidar_buffer.pop_front();
      return false;
    }
    meas.lidar_beg_time = time_buffer.front();
    lidar_end_time = meas.lidar_beg_time +
                     meas.lidar->points.back().curvature / double(1000);
    lidar_pushed = true;
  }

  if (last_timestamp_imu < lidar_end_time) {
    return false;
  }

  /*** push imu data, and pop from imu buffer ***/
  double imu_time = imu_buffer.front()->header.stamp.toSec();
  meas.imu.clear();
  while ((!imu_buffer.empty()) && (imu_time < lidar_end_time)) {
    imu_time = imu_buffer.front()->header.stamp.toSec();
    if (imu_time > lidar_end_time + 0.02)
      break;
    meas.imu.push_back(imu_buffer.front());
    imu_buffer.pop_front();
  }

  lidar_buffer.pop_front();
  time_buffer.pop_front();
  lidar_pushed = false;
  return true;
}

void publish_frame_world(const ros::Publisher &pubLaserCloudFullRes,
                         const int point_skip) {
  PointCloudXYZI::Ptr laserCloudFullRes(dense_map_en ? feats_undistort
                                                     : feats_down_body);
  int size = laserCloudFullRes->points.size();
  PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));
  for (int i = 0; i < size; i++) {
    RGBpointBodyToWorld(&laserCloudFullRes->points[i],
                        &laserCloudWorld->points[i]);
  }
  PointCloudXYZI::Ptr laserCloudWorldPub(new PointCloudXYZI);
  for (int i = 0; i < size; i += point_skip) {
    laserCloudWorldPub->points.push_back(laserCloudWorld->points[i]);
  }
  sensor_msgs::PointCloud2 laserCloudmsg;
  pcl::toROSMsg(*laserCloudWorldPub, laserCloudmsg);
  laserCloudmsg.header.stamp =
      ros::Time::now(); //.fromSec(last_timestamp_lidar);
  laserCloudmsg.header.frame_id = "camera_init";
  pubLaserCloudFullRes.publish(laserCloudmsg);
}

void publish_effect(const ros::Publisher& point_pub, const ros::Publisher& marker_pub) {
    static int count = 0;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    //* The match points are from the pv list, representing the point cloud in the world frame before the last optimization;
    //* therefore, there is an offset compared to the final published point cloud.
    for (const auto& effect_point : effect_points) {
        pcl::PointXYZRGB p;
        p.x = effect_point(0);
        p.y = effect_point(1);
        p.z = effect_point(2);
        p.r = 0; p.g = 255; p.b = 0;
        cloud->push_back(p);
    }
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "camera_init";
    cloud_msg.header.stamp = ros::Time::now();
    point_pub.publish(cloud_msg);

    // delete markers
    visualization_msgs::MarkerArray delete_marker_array;
    for (size_t i = 0; i < count; ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.header.stamp = ros::Time();
        marker.ns = "effect_arrow";
        marker.id = i;
        marker.action = visualization_msgs::Marker::DELETE;
        delete_marker_array.markers.push_back(marker);

        visualization_msgs::Marker marker_proj;
        marker_proj.header.frame_id = "camera_init";
        marker_proj.header.stamp = ros::Time();
        marker_proj.ns = "effect_proj";
        marker_proj.id = i;
        marker_proj.action = visualization_msgs::Marker::DELETE;
        delete_marker_array.markers.push_back(marker_proj);
    }
    marker_pub.publish(delete_marker_array);
    ros::Duration(0.05).sleep();

    // publish arrow markers
    visualization_msgs::MarkerArray marker_array;
    for (size_t i = 0; i < ptpl_list.size(); ++i) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "camera_init";
        marker.header.stamp = ros::Time();
        marker.ns = "effect_arrow";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        geometry_msgs::Point p1, p2;
        p1.x = effect_points[i](0);
        p1.y = effect_points[i](1);
        p1.z = effect_points[i](2);
        p2.x = ptpl_list[i].center(0);
        p2.y = ptpl_list[i].center(1);
        p2.z = ptpl_list[i].center(2);
        marker.points.push_back(p1);
        marker.points.push_back(p2);
        marker.scale.x = 0.03;
        marker.scale.y = 0.06;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration();
        marker_array.markers.push_back(marker);
    }
    marker_pub.publish(marker_array);
    ros::Duration(0.05).sleep();

    count = ptpl_list.size();

    for (size_t i = 0; i < ptpl_list.size(); ++i) {
      visualization_msgs::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.header.stamp = ros::Time();
      marker.ns = "effect_proj";
      marker.id = i;
      marker.type = visualization_msgs::Marker::ARROW;
      marker.action = visualization_msgs::Marker::ADD;

      geometry_msgs::Point p1, p2;
      // original point
      p1.x = effect_points[i](0);
      p1.y = effect_points[i](1);
      p1.z = effect_points[i](2);

      // plane parameters
      Eigen::Vector3d P = effect_points[i];
      Eigen::Vector3d n = ptpl_list[i].normal.normalized();
      Eigen::Vector3d C = ptpl_list[i].center;
      // projected point
      Eigen::Vector3d Q = P - n * ((P - C).dot(n));
      p2.x = Q(0);
      p2.y = Q(1);
      p2.z = Q(2);

      marker.points.push_back(p1);
      marker.points.push_back(p2);
      marker.scale.x = 0.02;
      marker.scale.y = 0.04;
      marker.scale.z = 0.08;
      marker.color.a = 1.0;
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.lifetime = ros::Duration();
      marker_array.markers.push_back(marker);
    }
    
    marker_pub.publish(marker_array);
}

void publish_no_effect(const ros::Publisher &pubLaserCloudNoEffect) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    //* match point is from the pv list, representing the point cloud in the world frame before the last optimization;
    //* therefore, there is an offset compared to the final published point cloud.
    for (const auto& point : non_match_list) {
        pcl::PointXYZRGB p;
        p.x = point(0);
        p.y = point(1);
        p.z = point(2);
        p.r = 255; p.g = 0; p.b = 0;
        cloud->push_back(p);
    }
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = "camera_init";
    cloud_msg.header.stamp = ros::Time::now();
    pubLaserCloudNoEffect.publish(cloud_msg);
}

void pub_all() {
  if(pub_all_plane_enable){
    voxelmap.pub_all_plane(voxel_map_all_plane_pub);
  }
  if(pub_all_plane_layer_color_enable){
    voxelmap.pub_all_plane_layer_color(voxel_map_all_plane_layer_color_pub);
  }
  if(pub_all_point_enable){
    voxelmap.pub_all_point(voxel_map_all_point_pub);
  }
  if(pub_grid_map_enable){
    voxelmap.pub_grid_map(voxel_map_grid_map_pub);
  }
}


// service callback function
bool publishAllCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
  pub_all_plane_layer_color_enable = true;
  pub_all_plane_enable = true;
  pub_all_point_enable = true;
  pub_all();
  pub_all_plane_layer_color_enable = false;
  pub_all_plane_enable = false;
  pub_all_point_enable = false;
  return true;
}

template <typename T> void set_posestamp(T &out) {
  out.position.x = state.pos_end(0);
  out.position.y = state.pos_end(1);
  out.position.z = state.pos_end(2);
  out.orientation.x = geoQuat.x;
  out.orientation.y = geoQuat.y;
  out.orientation.z = geoQuat.z;
  out.orientation.w = geoQuat.w;
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped) {
  odomAftMapped.header.frame_id = "camera_init";
  odomAftMapped.child_frame_id = "aft_mapped";
  odomAftMapped.header.stamp =
      ros::Time::now(); // ros::Time().fromSec(last_timestamp_lidar);
  set_posestamp(odomAftMapped.pose.pose);
  static tf::TransformBroadcaster br;
  tf::Transform transform;
  tf::Quaternion q;
  transform.setOrigin(
      tf::Vector3(state.pos_end(0), state.pos_end(1), state.pos_end(2)));
  q.setW(geoQuat.w);
  q.setX(geoQuat.x);
  q.setY(geoQuat.y);
  q.setZ(geoQuat.z);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp,
                                        "camera_init", "aft_mapped_voxelmap"));
  pubOdomAftMapped.publish(odomAftMapped);
}

void publish_mavros(const ros::Publisher &mavros_pose_publisher) {
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_odom_frame";
  set_posestamp(msg_body_pose.pose);
  mavros_pose_publisher.publish(msg_body_pose);
}

void publish_path(const ros::Publisher pubPath) {
  set_posestamp(msg_body_pose.pose);
  msg_body_pose.header.stamp = ros::Time::now();
  msg_body_pose.header.frame_id = "camera_init";
  path.poses.push_back(msg_body_pose);
  pubPath.publish(path);
}

void save_to_log(){
    // save tum format odom
    if (save_tum_flag && tum_odom_file.is_open())
    {
        Eigen::Quaterniond q(state.rot_end.matrix());

        tum_odom_file << std::fixed << std::setprecision(6) 
        << lidar_end_time << " " 
        << state.pos_end(0) << " "
        << state.pos_end(1) << " "
        << state.pos_end(2) << " "
        << q.x() << " "
        << q.y() << " "
        << q.z() << " "
        << q.w() 
        << std::endl;
        tum_odom_file.flush();
    }

    if(save_kitti_flag && kitti_odom_file.is_open()){
        MD(3, 4) T;
        T.block<3, 3>(0, 0) = state.rot_end.matrix();
        T.block<3, 1>(0, 3) = state.pos_end;
        kitti_odom_file << std::fixed << std::setprecision(6) 
        << T(0, 0) << " " << T(0, 1) << " " << T(0, 2) << " " << T(0, 3) << " "
        << T(1, 0) << " " << T(1, 1) << " " << T(1, 2) << " " << T(1, 3) << " "
        << T(2, 0) << " " << T(2, 1) << " " << T(2, 2) << " " << T(2, 3) << std::endl;
        kitti_odom_file.flush();
    }

    // memory log
    if(save_mem_flag && mem_file.is_open()){
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        double memory_usage_mb = usage.ru_maxrss / 1024.0;
        // std::cout << "Memory usage: " << memory_usage_mb << " MB" << std::endl;
        mem_file << std::fixed << std::setprecision(4) 
        << lidar_end_time << " "
        << memory_usage_mb << std::endl;
        mem_file.flush();
    }

    // time log (ms)
    if(save_time_flag && time_file.is_open()){
        time_file << std::fixed << std::setprecision(6) 
        << lidar_end_time << " "
        << time_vec_tmp[0] * 1000 << " "
        << time_vec_tmp[1] * 1000 << " "
        << time_vec_tmp[2] * 1000 << " "
        << time_vec_tmp[3] * 1000 << " "
        << time_vec_tmp[4] * 1000 << " "
        << time_vec_tmp[5] * 1000 << std::endl;
        time_file.flush();
    }
}

void update_time_log(){
    log_scan_num ++;
    time_vec_aver[0] += time_vec_tmp[0];
    time_vec_aver[1] += time_vec_tmp[1];
    time_vec_aver[2] += time_vec_tmp[2];
    time_vec_aver[3] += time_vec_tmp[3];
    time_vec_aver[4] += time_vec_tmp[4];
    time_vec_aver[5] += time_vec_tmp[5];
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "voxelMapping");
  ros::NodeHandle nh;

  double ranging_cov = 0.0;
  double angle_cov = 0.0;
  
  // config
  voxel_map_config config;

  // common params
  nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
  nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");

  // noise model params
  nh.param<double>("noise_model/ranging_cov", ranging_cov, 0.02);
  nh.param<double>("noise_model/angle_cov", angle_cov, 0.05);
  nh.param<double>("noise_model/gyr_cov_scale", gyr_cov_scale, 0.1);
  nh.param<double>("noise_model/acc_cov_scale", acc_cov_scale, 0.1);

  // imu params, current version does not support imu
  nh.param<bool>("imu/imu_en", imu_en, false);
  std::cout << "imu_en: " << imu_en << std::endl;
  nh.param<vector<double>>("imu/extrinsic_T", extrinT, vector<double>());
  nh.param<vector<double>>("imu/extrinsic_R", extrinR, vector<double>());

  // mapping algorithm params
  nh.param<double>("mapping/down_sample_size", filter_size_surf_min, 0.5);
  nh.param<int>("mapping/max_iteration", NUM_MAX_ITERATIONS, 4);
  
  nh.param<float>("mapping/voxel_size", config.voxel_size, 1.0);
  nh.param<int>("mapping/max_layer", config.max_layer, 2);
  nh.param<float>("mapping/plannar_threshold", config.planer_threshold, 0.01);

  nh.param<int>("mapping/max_points_size", config.max_points_size, 100);
  nh.param<int>("mapping/max_cov_points_size", config.max_cov_points_size, 100);
  nh.param<int>("mapping/max_root_points_size", config.max_root_points_size, 1000);
  nh.param<vector<int>>("mapping/layer_point_size", config.layer_point_size, vector<int>());
  nh.param<int>("mapping/update_size_threshold", config.update_size_threshold, 5);

  nh.param<int>("mapping/stop_reinit_threshold", config.stop_reinit_threshold, 1000);
  nh.param<int>("mapping/reinit_threshold", config.reinit_threshold, 100);
  nh.param<vector<int>>("mapping/reinit_size_vec", config.reinit_size_vec, vector<int>());

  nh.param<double>("mapping/update_inlier_distance_threshold", config.update_inlier_distance_threshold, 0.1);
  
  nh.param<int>("mapping/lru_cache_capacity", config.lru_capacity, 1000000);

  // ransac params
  nh.param<int>("ransac/ransac_max_iter", config.ransac_max_iter, 10);
  nh.param<int>("ransac/ransac_sample_num", config.ransac_sample_num, 4);
  nh.param<double>("ransac/ransac_inlier_distance_threshold", config.ransac_inlier_distance_threshold, 0.1);
  nh.param<double>("ransac/ransac_isplane_p_threshold", config.ransac_isplane_p_threshold, 0.8);
  nh.param<int>("ransac/sample_seed", config.sample_seed, 42);
  
  // valid check params
  nh.param<int>("valid_check/max_layer", config.valid_check_max_layer, 0);
  nh.param<int>("valid_check/min_points_size", config.valid_check_min_points_size, 10);
  nh.param<int>("valid_check/resolution", config.valid_check_resolution, 5);
  std::cout << "valid_check_resolution: " << config.valid_check_resolution << std::endl;

  // other params
  config.direct_match_layer = 1;
  config.pub_cache_en = false;
  config.pub_noupdate_voxel_en = false;

  // preprocess params
  nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
  nh.param<bool>("preprocess/calib_laser", calib_laser, false);
  nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
  nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
  nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 2);

  // visualization params
  nh.param<bool>("visualization/pub_update_plane", pub_update_plane, false);
  nh.param<int>("visualization/pub_max_voxel_layer", config.pub_max_voxel_layer, 0);
  nh.param<bool>("visualization/pub_point_cloud", publish_point_cloud, true);
  nh.param<int>("visualization/pub_point_cloud_skip", pub_point_cloud_skip, 1);
  nh.param<bool>("visualization/dense_map_enable", dense_map_en, false);
  nh.param<bool>("visualization/pub_effect", pub_effect, false);
  config.pub_effect_en = pub_effect;

  nh.param<bool>("visualization/pub_all_plane_enable", pub_all_plane_enable, false);
  voxel_map_all_plane_pub = nh.advertise<visualization_msgs::MarkerArray>("all_plane", 10000);
  nh.param<bool>("visualization/pub_all_plane_layer_color_enable", pub_all_plane_layer_color_enable, false);
  voxel_map_all_plane_layer_color_pub = nh.advertise<visualization_msgs::MarkerArray>("all_plane_color", 10000);
  nh.param<bool>("visualization/pub_all_point_enable", pub_all_point_enable, false);
  voxel_map_all_point_pub = nh.advertise<sensor_msgs::PointCloud2>("all_point", 10000);
  nh.param<bool>("visualization/pub_grid_map_enable", pub_grid_map_enable, false);
  config.pub_grid_map_en = pub_grid_map_enable;
  voxel_map_grid_map_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_map", 10000);


  // init VoxelMap
  voxelmap.init(config);

#ifdef DISABLE_LIVOX
  ros::Subscriber sub_pcl = nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
#else
  ros::Subscriber sub_pcl =
      p_pre->lidar_type == AVIA
          ? nh.subscribe(lid_topic, 200000, livox_pcl_cbk)
          : nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
#endif
  ros::Subscriber sub_imu;
  if (imu_en) {
    sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
  }

  ros::Publisher pubLaserCloudFullRes =
      nh.advertise<sensor_msgs::PointCloud2>("cloud_registered", 100);
  ros::Publisher pubOdomAftMapped =
      nh.advertise<nav_msgs::Odometry>("aft_mapped_to_init", 10);
  ros::Publisher pubPath = nh.advertise<nav_msgs::Path>("path", 10);
  ros::Publisher update_plane_pub =
      nh.advertise<visualization_msgs::MarkerArray>("planes", 10000);
  ros::Publisher point_effect_pub =
      nh.advertise<sensor_msgs::PointCloud2>("point_effected", 100);
  ros::Publisher marker_effect_pub =
      nh.advertise<visualization_msgs::MarkerArray>("marker_effected", 100);
  ros::Publisher point_no_effect_pub =
      nh.advertise<sensor_msgs::PointCloud2>("point_no_effected", 100);
  path.header.stamp = ros::Time::now();
  path.header.frame_id = "camera_init";

  ros::ServiceServer publish_all_service = nh.advertiseService("publish_all", publishAllCallback);

  // my log
  nh.param<string>("tum_file_path", tum_file_path, "./tum_odom.txt"); // tum数据集的路径
  nh.param<bool>("save_tum_flag", save_tum_flag, false);               // 是否保存tum数据集
  nh.param<string>("kitti_file_path", kitti_file_path, "./kitti.txt"); // kitti数据集的路径
  nh.param<bool>("save_kitti_flag", save_kitti_flag, false);
  nh.param<string>("mem_file_path", mem_file_path, "./mem.txt"); // mem数据集的路径
  nh.param<bool>("save_mem_flag", save_mem_flag, false);  
  nh.param<string>("time_file_path", time_file_path, "./time.txt"); // time数据集的路径
  nh.param<bool>("save_time_flag", save_time_flag, false);  
  nh.param<string>("time_aver_file_path", time_aver_file_path, "./time_aver.txt"); // time_aver数据集的路径

  // create file if needed
  if (save_tum_flag) {
      ROS_INFO("[log] tum odom will be saved to %s", tum_file_path.c_str());
      tum_odom_file.open(tum_file_path, std::ios::trunc);
  }
  if (save_kitti_flag) {
      ROS_INFO("[log] kitti odom will be saved to %s", kitti_file_path.c_str());
      kitti_odom_file.open(kitti_file_path, std::ios::trunc);
  }
  if (save_mem_flag) {
      ROS_INFO("[log] memory usage will be saved to %s", mem_file_path.c_str());
      mem_file.open(mem_file_path, std::ios::trunc);
  }
  if (save_time_flag) {
      ROS_INFO("[log] time will be saved to %s", time_file_path.c_str());
      time_file.open(time_file_path, std::ios::trunc);
      time_aver_file.open(time_aver_file_path, std::ios::trunc);
  }

  /*** variables definition ***/
  VD(DIM_STATE) solution;
  MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
  V3D rot_add, t_add;
  StatesGroup state_propagat;
  PointType pointOri, pointSel, coeff;
  PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));  // 一帧中，每个有效点匹配平面的法向量（存在XYZ中）和点到平面距离（存在I中）
  int frame_num = 0;
  double deltaT, deltaR, aver_time_consu = 0;
  bool flg_EKF_inited, flg_EKF_converged, EKF_stop_flg = 0,
                                          is_first_frame = true;
  downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min,
                                 filter_size_surf_min);

  shared_ptr<ImuProcess> p_imu(new ImuProcess());
  p_imu->imu_en = imu_en;
  Eigen::Vector3d extT = Eigen::Vector3d::Zero();
  Eigen::Matrix3d extR = Eigen::Matrix3d::Identity();
  if (extrinT.size() == 3) {
    extT << extrinT[0], extrinT[1], extrinT[2];
  } else if (!extrinT.empty()) {
    ROS_WARN("imu/extrinsic_T size=%zu, expect 3; using zeros", extrinT.size());
  }
  if (extrinR.size() == 9) {
    extR << extrinR[0], extrinR[1], extrinR[2], extrinR[3], extrinR[4],
        extrinR[5], extrinR[6], extrinR[7], extrinR[8];
  } else if (!extrinR.empty()) {
    ROS_WARN("imu/extrinsic_R size=%zu, expect 9; using identity", extrinR.size());
  }
  Lidar_offset_to_IMU = extT;
  Lidar_rot_to_IMU = extR;
  p_imu->set_extrinsic(extT, extR);
  ROS_INFO("LiDAR-IMU extrinsic T: [%.6f, %.6f, %.6f]",
           extT(0), extT(1), extT(2));
  ROS_INFO_STREAM("LiDAR-IMU extrinsic R:\n" << extR);

  // Current version do not support imu.
  if (imu_en) {
    std::cout << "use imu" << std::endl;
  } else {
    std::cout << "no imu" << std::endl;
  }

  p_imu->set_gyr_cov_scale(V3D(gyr_cov_scale, gyr_cov_scale, gyr_cov_scale));
  p_imu->set_acc_cov_scale(V3D(acc_cov_scale, acc_cov_scale, acc_cov_scale));
  p_imu->set_gyr_bias_cov(V3D(0.00001, 0.00001, 0.00001));
  p_imu->set_acc_bias_cov(V3D(0.00001, 0.00001, 0.00001));

  G.setZero();
  H_T_H.setZero();
  I_STATE.setIdentity();

  std::string bag_path;
  lio_offline::BagFeeder bag_feeder;
  const bool offline_mode = lio_offline::getBagPath(nh, bag_path) &&
                            bag_feeder.open(bag_path, lid_topic, imu_topic);

  signal(SIGINT, SigHandle);
  ros::Rate rate(5000);
  bool status = ros::ok();

  // for Plane Map
  bool init_map = false;
  last_rot << 1, 0, 0, 0, 1, 0, 0, 0, 1;  // dont use

  while (status) {
    if (flg_exit) break;

    bool have_measure = false;

    if (offline_mode) {

        bool fed = bag_feeder.feedUntilLidar(

            [](const sensor_msgs::Imu::ConstPtr &msg) { imu_cbk(msg); },

            [](const sensor_msgs::PointCloud2::ConstPtr &msg) { standard_pcl_cbk(msg); });

        have_measure = sync_packages(Measures);

        if (!have_measure && !fed && bag_feeder.done()) break;

    } else {

        ros::spinOnce();

        have_measure = sync_packages(Measures);

    }

    if(have_measure) {
      // std::cout << "sync once" << std::endl;
      double t_all_start = omp_get_wtime();
      if (flg_reset) {
        ROS_WARN("reset when rosbag play back");
        p_imu->Reset();
        flg_reset = false;
        continue;
      }
      std::cout << "scanIdx:" << scanIdx << std::endl;
      double t0, t1, t2, t3, t4, t5, match_start, match_time, solve_start,
          svd_time;
      match_time = 0;
      solve_time = 0;
      svd_time = 0;

      auto undistort_start = std::chrono::high_resolution_clock::now();
      double t_imu_start = omp_get_wtime();
      p_imu->Process(Measures, state, feats_undistort);
      double t_imu_end = omp_get_wtime();
      time_vec_tmp[0] = t_imu_end - t_imu_start;

      // * =====Process==================
      auto undistort_end = std::chrono::high_resolution_clock::now();
      auto undistort_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              undistort_end - undistort_start)
              .count() *
          1000;
      if (calib_laser) {
        // calib the vertical angle for kitti dataset
        for (size_t i = 0; i < feats_undistort->size(); i++) {
          PointType pi = feats_undistort->points[i];
          double range = sqrt(pi.x * pi.x + pi.y * pi.y + pi.z * pi.z);
          double calib_vertical_angle = deg2rad(0.15);
          double vertical_angle = asin(pi.z / range) + calib_vertical_angle;
          double horizon_angle = atan2(pi.y, pi.x);
          pi.z = range * sin(vertical_angle);
          double project_len = range * cos(vertical_angle);
          pi.x = project_len * cos(horizon_angle);
          pi.y = project_len * sin(horizon_angle);
          feats_undistort->points[i] = pi;
        }
      }
      state_propagat = state;

      if (is_first_frame) {
        first_lidar_time = Measures.lidar_beg_time;
        is_first_frame = false;
      }

      if (feats_undistort->empty() || (feats_undistort == NULL)) {
        p_imu->first_lidar_time = first_lidar_time;
        cout << "FAST-LIO not ready" << endl;
        continue;
      }

      // later 待看用法，默认INIT_TIME为0，直接进入true？
      flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME
                           ? false
                           : true;
      
      // init map
      if (flg_EKF_inited && !init_map) {
        pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
            new pcl::PointCloud<pcl::PointXYZI>);
        Eigen::Quaterniond q(state.rot_end);
        transformLidar(state, feats_undistort, world_lidar);
        std::vector<pointWithCov> pv_list;
        for (size_t i = 0; i < world_lidar->size(); i++) {
          pointWithCov pv;
          pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
              world_lidar->points[i].z;
          V3D point_this(feats_undistort->points[i].x,
                         feats_undistort->points[i].y,
                         feats_undistort->points[i].z);
          // if z=0, error will occur in calcBodyCov. To be solved
          if (point_this[2] == 0) {
            point_this[2] = 0.001;
          }
          M3D cov;
          calcBodyCov(point_this, ranging_cov, angle_cov, cov);

          // lidar-frame cov/point -> IMU body frame
          cov = Lidar_rot_to_IMU * cov * Lidar_rot_to_IMU.transpose();
          point_this = Lidar_rot_to_IMU * point_this + Lidar_offset_to_IMU;
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          // * calculate cov of point in world frame
          cov = state.rot_end * cov * state.rot_end.transpose() +
                state.rot_end * (-point_crossmat) * state.cov.block<3, 3>(0, 0) *
                    (-point_crossmat).transpose() * state.rot_end.transpose() +
                state.cov.block<3, 3>(3, 3);
          pv.cov = cov;
          pv_list.push_back(pv);
        }

        // init map
        voxelmap.build(pv_list);

        feats_down_body->clear();
        feats_down_body->points.resize(feats_undistort->size());
        for (size_t i = 0; i < feats_undistort->size(); i++) {
          feats_down_body->points[i] = feats_undistort->points[i];
        }
        if (publish_point_cloud) {
          publish_frame_world(pubLaserCloudFullRes, pub_point_cloud_skip);
        }

        std::cout << "build voxel map" << std::endl;

        // my log
        save_to_log();

        // pub
        pub_all();

        scanIdx++;
        if (pub_update_plane) {
          voxelmap.pub_update_plane(update_plane_pub);
        }
        init_map = true;
        continue;
      }

      /*** downsample the feature points in a scan ***/
      auto t_downsample_start = std::chrono::high_resolution_clock::now();
      double t_downsample_start_tmp = omp_get_wtime();
      downSizeFilterSurf.setInputCloud(feats_undistort);
      downSizeFilterSurf.filter(*feats_down_body);
      double t_downsample_end_tmp = omp_get_wtime();
      time_vec_tmp[1] = t_downsample_end_tmp - t_downsample_start_tmp;
      auto t_downsample_end = std::chrono::high_resolution_clock::now();
      std::cout << " feats size:" << feats_undistort->size()
                << ", down size:" << feats_down_body->size() << std::endl;
      auto t_downsample =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              t_downsample_end - t_downsample_start)
              .count() *
          1000;

      //* sort points by timestamp
      sort(feats_down_body->points.begin(), feats_down_body->points.end(),
           time_list);

      int rematch_num = 0;
      bool nearest_search_en = true;
      double total_residual;

      scan_match_time = 0.0;

      std::vector<M3D> body_var;
      std::vector<M3D> crossmat_list;

      /*** iterated state estimation ***/
      auto calc_point_cov_start = std::chrono::high_resolution_clock::now();
      for (size_t i = 0; i < feats_down_body->size(); i++) {
        V3D point_this(feats_down_body->points[i].x,
                       feats_down_body->points[i].y,
                       feats_down_body->points[i].z);
        if (point_this[2] == 0) {
          point_this[2] = 0.001;
        }
        M3D cov;
        // calculate cov of point in lidar frame, then lift to IMU body
        calcBodyCov(point_this, ranging_cov, angle_cov, cov);
        cov = Lidar_rot_to_IMU * cov * Lidar_rot_to_IMU.transpose();
        point_this = Lidar_rot_to_IMU * point_this + Lidar_offset_to_IMU;
        M3D point_crossmat;
        point_crossmat << SKEW_SYM_MATRX(point_this);
        crossmat_list.push_back(point_crossmat);
        body_var.push_back(cov);
      }
      auto calc_point_cov_end = std::chrono::high_resolution_clock::now();
      double calc_point_cov_time =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              calc_point_cov_end - calc_point_cov_start)
              .count() *
          1000;

      double t_match_start = omp_get_wtime();
      for (iterCount = 0; iterCount < NUM_MAX_ITERATIONS; iterCount++) {
        laserCloudOri->clear();
        laserCloudNoeffect->clear();
        corr_normvect->clear();
        total_residual = 0.0;

        std::vector<double> r_list;
        ptpl_list.clear();
        non_match_list.clear();
        effect_points.clear();
        /** LiDAR match based on 3 sigma criterion **/

        vector<pointWithCov> pv_list;
        // std::vector<M3D> var_list;
        pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
            new pcl::PointCloud<pcl::PointXYZI>);
        transformLidar(state, feats_down_body, world_lidar);
        for (size_t i = 0; i < feats_down_body->size(); i++) {
          pointWithCov pv;
          pv.point << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
          M3D cov = body_var[i];
          M3D point_crossmat = crossmat_list[i];
          M3D rot_var = state.cov.block<3, 3>(0, 0);
          M3D t_var = state.cov.block<3, 3>(3, 3);
          // old version
          // cov = state.rot_end * cov * state.rot_end.transpose() +
          //       (-point_crossmat) * rot_var * (-point_crossmat.transpose()) +
          //       t_var;
          // * change 3sigma 判断时需要
          cov = state.rot_end * cov * state.rot_end.transpose() +
                state.rot_end * (-point_crossmat) * rot_var * (-point_crossmat.transpose()) * state.rot_end.transpose() +
                t_var;
          pv.cov = cov;
          pv_list.push_back(pv);
          // var_list.push_back(cov);
        }
        auto scan_match_time_start = std::chrono::high_resolution_clock::now();
        // match
        std::vector<int> useful_index;
        voxelmap.build_residual_omp(pv_list, config.max_layer, ptpl_list, useful_index, non_match_list);

        auto scan_match_time_end = std::chrono::high_resolution_clock::now();

        effct_feat_num = 0;
        for (int i = 0; i < ptpl_list.size(); i++) {
          PointType pi_body;
          PointType pi_world;
          PointType pl;
          pi_body.x = feats_down_body->points[useful_index[i]].x;
          pi_body.y = feats_down_body->points[useful_index[i]].y;
          pi_body.z = feats_down_body->points[useful_index[i]].z;
          pointBodyToWorld(&pi_body, &pi_world);
          effect_points.push_back({pi_world.x, pi_world.y, pi_world.z});
          pl.x = ptpl_list[i].normal(0);
          pl.y = ptpl_list[i].normal(1);
          pl.z = ptpl_list[i].normal(2);
          effct_feat_num++;

          float dis = (pi_world.x * pl.x + pi_world.y * pl.y + pi_world.z * pl.z + ptpl_list[i].d);
          pl.intensity = dis;
          laserCloudOri->push_back(pi_body);
          corr_normvect->push_back(pl); // plane normal vector
          total_residual += fabs(dis);
        }

        res_mean_last = total_residual / effct_feat_num;
        scan_match_time +=
            std::chrono::duration_cast<std::chrono::duration<double>>(
                scan_match_time_end - scan_match_time_start)
                .count() *
            1000;

        cout << "[ Matching ]: Time:"
             << std::chrono::duration_cast<std::chrono::duration<double>>(
                    scan_match_time_end - scan_match_time_start)
                        .count() *
                    1000
             << " ms  Effective feature num: " << effct_feat_num
             << " All num:" << feats_down_body->size() << "  res_mean_last "
             << res_mean_last << endl;

        auto t_solve_start = std::chrono::high_resolution_clock::now();

        /*** Computation of Measuremnt Jacobian matrix H and measurents vector
         * ***/
        MatrixXd Hsub(effct_feat_num, 6);
        MatrixXd Hsub_T_R_inv(6, effct_feat_num);
        VectorXd R_inv(effct_feat_num);
        VectorXd meas_vec(effct_feat_num);

        for (int i = 0; i < effct_feat_num; i++) {
          const PointType &laser_p = laserCloudOri->points[i];
          V3D point_lidar(laser_p.x, laser_p.y, laser_p.z); // lidar frame
          M3D cov;
          if (calib_laser) {
            calcBodyCov(point_lidar, ranging_cov, CALIB_ANGLE_COV, cov);
          } else {
            // calculate cov of point in lidar frame
            if(point_lidar[2] == 0) {
              point_lidar[2] = 0.001;
              calcBodyCov(point_lidar, ranging_cov, angle_cov, cov);
              point_lidar[2] = 0;
            }
            else {
              calcBodyCov(point_lidar, ranging_cov, angle_cov, cov);
            }
          }

          // lift lidar cov/point to IMU body, then to world for observation noise
          cov = Lidar_rot_to_IMU * cov * Lidar_rot_to_IMU.transpose();
          V3D point_this = Lidar_rot_to_IMU * point_lidar + Lidar_offset_to_IMU;
          cov = state.rot_end * cov * state.rot_end.transpose();
          M3D point_crossmat;
          point_crossmat << SKEW_SYM_MATRX(point_this);
          const PointType &norm_p = corr_normvect->points[i];
          // current point matching plane normal vector
          V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
          // point in world frame
          V3D point_world = state.rot_end * point_this + state.pos_end;
          // /*** get the normal vector of closest surface/corner ***/
          Eigen::Matrix<double, 1, 6> J_nq;
          J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
          J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
          double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();
          R_inv(i) = 1.0 / (sigma_l + norm_vec.transpose() * cov * norm_vec);
          laserCloudOri->points[i].intensity = sqrt(R_inv(i));
          laserCloudOri->points[i].normal_x = corr_normvect->points[i].intensity;
          laserCloudOri->points[i].normal_y = sqrt(sigma_l);
          laserCloudOri->points[i].normal_z = sqrt(norm_vec.transpose() * cov * norm_vec);
          laserCloudOri->points[i].curvature = sqrt(sigma_l + norm_vec.transpose() * cov * norm_vec);

          /*** calculate the Measuremnt Jacobian matrix H ***/
          // A = [p_imu]_x * R^T * n  (p in IMU body frame)
          V3D A(point_crossmat * state.rot_end.transpose() * norm_vec);
          Hsub.row(i) << VEC_FROM_ARRAY(A), norm_p.x, norm_p.y, norm_p.z;
          Hsub_T_R_inv.col(i) << A[0] * R_inv(i), A[1] * R_inv(i),
              A[2] * R_inv(i), norm_p.x * R_inv(i), norm_p.y * R_inv(i),
              norm_p.z * R_inv(i);
          meas_vec(i) = -norm_p.intensity;
        }
        MatrixXd K(DIM_STATE, effct_feat_num);

        EKF_stop_flg = false;
        flg_EKF_converged = false;

        /*** Iterative Kalman Filter Update ***/
        if (!flg_EKF_inited) {
          cout << "||||||||||Initiallizing LiDar||||||||||" << endl;
          /*** only run in initialization period ***/
          MatrixXd H_init(MD(9, DIM_STATE)::Zero());
          MatrixXd z_init(VD(9)::Zero());
          H_init.block<3, 3>(0, 0) = M3D::Identity();
          H_init.block<3, 3>(3, 3) = M3D::Identity();
          H_init.block<3, 3>(6, 15) = M3D::Identity();
          z_init.block<3, 1>(0, 0) = -Log(state.rot_end);
          z_init.block<3, 1>(0, 0) = -state.pos_end;

          auto H_init_T = H_init.transpose();
          auto &&K_init =
              state.cov * H_init_T *
              (H_init * state.cov * H_init_T + 0.0001 * MD(9, 9)::Identity())
                  .inverse();
          solution = K_init * z_init;

          state.resetpose();
          EKF_stop_flg = true;
        } else {
          auto &&Hsub_T = Hsub.transpose();
          H_T_H.block<6, 6>(0, 0) = Hsub_T_R_inv * Hsub;
          MD(DIM_STATE, DIM_STATE) &&K_1 =
              (H_T_H + (state.cov).inverse()).inverse();
          K = K_1.block<DIM_STATE, 6>(0, 0) * Hsub_T_R_inv;
          auto vec = state_propagat - state;
          solution = K * meas_vec + vec - K * Hsub * vec.block<6, 1>(0, 0);

          int minRow, minCol;
          if (0) // if(V.minCoeff(&minRow, &minCol) < 1.0f)
          {
            VD(6) V = H_T_H.block<6, 6>(0, 0).eigenvalues().real();
            cout << "!!!!!! Degeneration Happend, eigen values: "
                 << V.transpose() << endl;
            EKF_stop_flg = true;
            solution.block<6, 1>(9, 0).setZero();
          }

          state += solution;

          rot_add = solution.block<3, 1>(0, 0);
          t_add = solution.block<3, 1>(3, 0);

          if ((rot_add.norm() * 57.3 < 0.01) && (t_add.norm() * 100 < 0.015)) {
            flg_EKF_converged = true;
          }

          deltaR = rot_add.norm() * 57.3;
          deltaT = t_add.norm() * 100;
        }
        euler_cur = RotMtoEuler(state.rot_end);
        /*** Rematch Judgement ***/
        nearest_search_en = false;
        if (flg_EKF_converged ||
            ((rematch_num == 0) && (iterCount == (NUM_MAX_ITERATIONS - 2)))) {
          nearest_search_en = true;
          rematch_num++;
        }

        /*** Convergence Judgements and Covariance Update ***/
        if (!EKF_stop_flg &&
            (rematch_num >= 2 || (iterCount == NUM_MAX_ITERATIONS - 1))) {
          if (flg_EKF_inited) {
            /*** Covariance Update ***/
            G.setZero();
            G.block<DIM_STATE, 6>(0, 0) = K * Hsub;
            state.cov = (I_STATE - G) * state.cov;
            total_distance += (state.pos_end - position_last).norm();
            position_last = state.pos_end;

            geoQuat = tf::createQuaternionMsgFromRollPitchYaw(
                euler_cur(0), euler_cur(1), euler_cur(2));

            VD(DIM_STATE) K_sum = K.rowwise().sum();
            VD(DIM_STATE) P_diag = state.cov.diagonal();
          }
          EKF_stop_flg = true;
        }
        auto t_solve_end = std::chrono::high_resolution_clock::now();
        solve_time += std::chrono::duration_cast<std::chrono::duration<double>>(
                          t_solve_end - t_solve_start)
                          .count() *
                      1000;

        cout << "[ EKF ]: Time:"
             << std::chrono::duration_cast<std::chrono::duration<double>>(
                    t_solve_end - t_solve_start)
                        .count() *
                    1000
             << " ms" << endl;

        if (EKF_stop_flg)
          break;
      }
      double t_match_end = omp_get_wtime();
      time_vec_tmp[2] = t_match_end - t_match_start;
      std::cout << "[ Solve ]: Time:" << time_vec_tmp[2] * 1000 << "ms" << std::endl;

      /*** add the  points to the voxel map ***/
      double t_map_start = omp_get_wtime();
      pcl::PointCloud<pcl::PointXYZI>::Ptr world_lidar(
          new pcl::PointCloud<pcl::PointXYZI>);
      transformLidar(state, feats_down_body, world_lidar);
      std::vector<pointWithCov> pv_list;
      for (size_t i = 0; i < world_lidar->size(); i++) {
        pointWithCov pv;
        pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
            world_lidar->points[i].z;
        M3D point_crossmat = crossmat_list[i];
        M3D cov = body_var[i];
        // old version
        // cov = state.rot_end * cov * state.rot_end.transpose() +
        //       (-point_crossmat) * state.cov.block<3, 3>(0, 0) *
        //           (-point_crossmat).transpose() +
        //       state.cov.block<3, 3>(3, 3);
        // * change
        cov = state.rot_end * cov * state.rot_end.transpose() +
              state.rot_end * (-point_crossmat) * state.cov.block<3, 3>(0, 0) *
                  (-point_crossmat).transpose() * state.rot_end.transpose() +
              state.cov.block<3, 3>(3, 3);
        pv.cov = cov;
        pv_list.push_back(pv);
      }
      std::sort(pv_list.begin(), pv_list.end(), var_contrast);
      
      // update map
      voxelmap.update(pv_list);
      
      // update LRU cache
      voxelmap.lru_cache_update();

      //* if lru_cache is full, delete the oldest voxel
      struct rusage usage;
      getrusage(RUSAGE_SELF, &usage);
      float memory_usage_mb = usage.ru_maxrss / 1024.0;
      cout << "lru_cache size: " << voxelmap.lru_cache.size() << endl;
      cout << "rvoxelmap size: " << voxelmap.feat_map.size() << endl;
      cout << "memory usage: " << memory_usage_mb << " MB" << endl;

      double t_map_end = omp_get_wtime();
      time_vec_tmp[3] = t_map_end - t_map_start;

      double t_all_end = omp_get_wtime();
      time_vec_tmp[5] = t_all_end - t_all_start;
      time_vec_tmp[4] = time_vec_tmp[5] - time_vec_tmp[0] - time_vec_tmp[1] - time_vec_tmp[2] - time_vec_tmp[3];

      total_time = time_vec_tmp[5];
      /******* Publish functions:  *******/
      publish_odometry(pubOdomAftMapped);
      publish_path(pubPath);
      tf::Transform transform;
      tf::Quaternion q;
      transform.setOrigin(
          tf::Vector3(state.pos_end(0), state.pos_end(1), state.pos_end(2)));
      q.setW(geoQuat.w);
      q.setX(geoQuat.x);
      q.setY(geoQuat.y);
      q.setZ(geoQuat.z);
      transform.setRotation(q);
      transformLidar(state, feats_down_body, world_lidar);
      sensor_msgs::PointCloud2 pub_cloud;
      pcl::toROSMsg(*world_lidar, pub_cloud);
      pub_cloud.header.stamp =
          ros::Time::now(); //.fromSec(last_timestamp_lidar);
      pub_cloud.header.frame_id = "camera_init";
      if (publish_point_cloud) {
        publish_frame_world(pubLaserCloudFullRes, pub_point_cloud_skip);
      }

      if (pub_update_plane) {
        voxelmap.pub_update_plane(update_plane_pub);
      }

      if (pub_effect) {
        publish_effect(point_effect_pub, marker_effect_pub);
        publish_no_effect(point_no_effect_pub);
      }

      frame_num++;
      mean_raw_points = mean_raw_points * (frame_num - 1) / frame_num +
                        (double)(feats_undistort->size()) / frame_num;
      mean_ds_points = mean_ds_points * (frame_num - 1) / frame_num +
                       (double)(feats_down_body->size()) / frame_num;
      mean_effect_points = mean_effect_points * (frame_num - 1) / frame_num +
                           (double)effct_feat_num / frame_num;

      undistort_time_mean = undistort_time_mean * (frame_num - 1) / frame_num +
                            (undistort_time) / frame_num;
      down_sample_time_mean =
          down_sample_time_mean * (frame_num - 1) / frame_num +
          (t_downsample) / frame_num;
      calc_cov_time_mean = calc_cov_time_mean * (frame_num - 1) / frame_num +
                           (calc_point_cov_time) / frame_num;
      scan_match_time_mean =
          scan_match_time_mean * (frame_num - 1) / frame_num +
          (scan_match_time) / frame_num;
      ekf_solve_time_mean = ekf_solve_time_mean * (frame_num - 1) / frame_num +
                            (solve_time) / frame_num;
      map_update_time_mean =
          map_update_time_mean * (frame_num - 1) / frame_num +
          (map_incremental_time) / frame_num;

      aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num +
                        (total_time) / frame_num;

      time_log_counter++;


      save_to_log();
      update_time_log();


      cout << "[ Time average ]: "
            << " imu preprocess: " << time_vec_aver[0] / log_scan_num * 1000 << " ms" << std::endl;
      cout << "[ Time average ]: "
            << " down sample: " << time_vec_aver[1] / log_scan_num * 1000 << " ms" << std::endl;
      cout << "[ Time average ]: "
            << " kf update: " << time_vec_aver[2] / log_scan_num * 1000 << " ms" << std::endl;
      cout << "[ Time average ]: "
            << " map update: " << time_vec_aver[3] / log_scan_num * 1000 << " ms" << std::endl;
      cout << "[ Time average ]: "
            << " other: " << time_vec_aver[4] / log_scan_num * 1000 << " ms" << std::endl;
      cout << "[ Time average ]: "
            << " total: " << time_vec_aver[5] / log_scan_num * 1000 << " ms" << std::endl;

      cout << "[ Time ]: "
            << " scan match: " << time_vec_tmp[2] * 1000 << " ms" << std::endl;
      cout << "[ Time ]: "
            << " map update: " << time_vec_tmp[3] * 1000 << " ms" << std::endl;
      cout << "[ Time ]: "
            << " total: " << time_vec_tmp[5] * 1000 << " ms" << std::endl;

      cout << "=================================" << std::endl;

      pub_all();

      scanIdx++;
    }
    status = ros::ok();

    if (!offline_mode) rate.sleep();
  }

  // my log
  if (tum_odom_file.is_open()) tum_odom_file.close();
  if (kitti_odom_file.is_open()) kitti_odom_file.close();
  if (mem_file.is_open()) mem_file.close();
  if (time_file.is_open()) time_file.close();
  if (save_time_flag && time_aver_file.is_open()) {
      time_aver_file << std::fixed << std::setprecision(3)  // 设置保留3位小数
      << "imu preprocess time: " << time_vec_aver[0] / log_scan_num * 1000 << " ms" << endl;
      time_aver_file << "downsample time: " << time_vec_aver[1] / log_scan_num * 1000 << " ms" << endl;
      time_aver_file << "kf update time: " << time_vec_aver[2] / log_scan_num * 1000 << " ms" << endl;
      time_aver_file << "map update time: " << time_vec_aver[3] / log_scan_num * 1000 << " ms" << endl;
      time_aver_file << "other time: " << time_vec_aver[4] / log_scan_num * 1000 << " ms" << endl;
      time_aver_file << "total time: " << time_vec_aver[5] / log_scan_num * 1000 << " ms" << endl;
      time_aver_file << "scan num: " << log_scan_num << endl;
      time_aver_file.close();
  }


  return 0;
}
