#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <sys/resource.h>
#include "../IMU_Processing.hpp"
#include "voxel_map_util.hpp"
#include <std_srvs/Empty.h>

using namespace std;

string pcd_file;

VoxelMap voxelmap;
voxel_map_config config;
double voxel_size;

bool downsample_enable = true;
double voxel_filter_size;
bool choose_range = false;
vector<int> range_list;

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

// publish topic that true in config file
bool publishCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    pub_all();
    return true;
}

// publish all topic
bool publishAllCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    bool temp_pub_all_plane_layer_color_enable = pub_all_plane_layer_color_enable;
    bool temp_pub_all_plane_enable = pub_all_plane_enable;
    bool temp_pub_all_point_enable = pub_all_point_enable;
    pub_all_plane_layer_color_enable = true;
    pub_all_plane_enable = true;
    pub_all_point_enable = true;

    pub_all();

    pub_all_plane_layer_color_enable = temp_pub_all_plane_layer_color_enable;
    pub_all_plane_enable = temp_pub_all_plane_enable;
    pub_all_point_enable = temp_pub_all_point_enable;
    return true;
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "voxel_map_test");
    ros::NodeHandle nh;

    nh.param<string>("pcd_file", pcd_file, "");

    // voxelmap config
    nh.param<int>("mapping/max_layer", config.max_layer, 2);
    nh.param<float>("mapping/voxel_size", config.voxel_size, 1.0);
    nh.param<float>("mapping/plannar_threshold", config.planer_threshold, 0.01);

    nh.param<int>("mapping/max_points_size", config.max_points_size, 100);
    nh.param<int>("mapping/max_cov_points_size", config.max_cov_points_size, 1000);
    nh.param<int>("mapping/max_root_points_size", config.max_root_points_size, 10000);
    nh.param<vector<int>>("mapping/layer_point_size", config.layer_point_size, vector<int>());
    nh.param<int>("mapping/update_size_threshold", config.update_size_threshold, 5);

    nh.param<int>("mapping/stop_reinit_threshold", config.stop_reinit_threshold, 1000);
    nh.param<int>("mapping/reinit_threshold", config.reinit_threshold, 100);
    nh.param<vector<int>>("mapping/reinit_size_vec", config.reinit_size_vec, vector<int>());
   
    nh.param<int>("mapping/lru_cache_capacity", config.lru_capacity, 1000000);

    // ransac params
    nh.param<int>("ransac/ransac_max_iter", config.ransac_max_iter, 10);
    nh.param<int>("ransac/ransac_sample_num", config.ransac_sample_num, 4);
    nh.param<double>("ransac/ransac_inlier_distance_threshold", config.ransac_inlier_distance_threshold, 0.1);
    nh.param<double>("ransac/ransac_isplane_p_threshold", config.ransac_isplane_p_threshold, 0.8);

    // valid check params
    nh.param<int>("valid_check/max_layer", config.valid_check_max_layer, 0);
    nh.param<int>("valid_check/min_points_size", config.valid_check_min_points_size, 10);
    nh.param<int>("valid_check/resolution", config.valid_check_resolution, 4);

    nh.param<int>("visualization/pub_max_voxel_layer", config.pub_max_voxel_layer, 5);
    nh.param<bool>("visualization/pub_all_plane_enable", pub_all_plane_enable, true);
    nh.param<bool>("visualization/pub_all_plane_layer_color_enable", pub_all_plane_layer_color_enable, true);
    nh.param<bool>("visualization/pub_all_point_enable", pub_all_point_enable, true);
    nh.param<bool>("visualization/pub_grid_map_enable", pub_grid_map_enable, true);
    config.pub_grid_map_en = pub_grid_map_enable;

    nh.param<bool>("downsample/enable", downsample_enable, true);
    nh.param<double>("downsample/voxel_size", voxel_filter_size, 0.1);  // 默认体素大小为0.1m
    voxel_size = config.voxel_size;
    
    nh.param<bool>("choose_range/enable", choose_range, false);
    nh.param<vector<int>>("choose_range/range_list", range_list, vector<int>());

    // publish topic
    ros::Publisher point_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("map_origin", 1);
    voxel_map_all_plane_pub = nh.advertise<visualization_msgs::MarkerArray>("all_plane", 10000);
    voxel_map_all_plane_layer_color_pub = nh.advertise<visualization_msgs::MarkerArray>("all_plane_color", 10000);
    voxel_map_all_point_pub = nh.advertise<sensor_msgs::PointCloud2>("all_point", 10000);
    voxel_map_grid_map_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_map", 10000);
    // service
    ros::ServiceServer publish_again_srv = nh.advertiseService("/publish_again", publishCallback);
    ros::ServiceServer publish_all_srv = nh.advertiseService("/publish_all", publishAllCallback);

    voxelmap.init(config);
    
    // ====== read pcd file =========================
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *cloud) == -1) {
        ROS_ERROR("Couldn't read file %s", pcd_file.c_str());
        return -1;
    }
    ROS_INFO("Loaded %d data points from %s", cloud->width * cloud->height, pcd_file.c_str());
    ROS_INFO("Original point size: %ld", cloud->points.size());
    std::cout << "=================================" << std::endl;

    // downsample
    if (downsample_enable) {
        pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
        voxel_filter.setInputCloud(cloud);
        voxel_filter.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);
        voxel_filter.filter(*cloud);
        ROS_INFO("After voxel filtering point size: %ld", cloud->points.size());
    }

    if(choose_range){
        // filter point cloud, only keep points in the specified range
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (const auto& point : cloud->points) {
            if (point.x >= range_list[0] * voxel_size && point.x <= range_list[1] * voxel_size && 
                point.y >= range_list[2] * voxel_size && point.y <= range_list[3] * voxel_size && 
                point.z >= range_list[4] * voxel_size && point.z <= range_list[5] * voxel_size) {
                filtered_cloud->points.push_back(point);
            }
        }
        // replace original point cloud with filtered point cloud
        cloud = filtered_cloud;
        ROS_INFO("After coordinate filtering: %ld points", cloud->points.size());
    }

    // convert point into pv
    std::vector<pointWithCov> pv_list;
    for (size_t i = 0; i < cloud->points.size(); i++) {
        pointWithCov pv; 
        // ! pv.point is the point in the world frame
        pv.point << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
        pv.cov = M3D::Identity() * 0.001;
        pv_list.push_back(pv);
    }
    ROS_WARN("pv_list size:%ld", pv_list.size());

    // * build voxel map
    auto t_buildmap_start = std::chrono::high_resolution_clock::now();
    voxelmap.build(pv_list);
    auto t_buildmap_end = std::chrono::high_resolution_clock::now();
    double t_buildmap = std::chrono::duration_cast<std::chrono::duration<double>>(t_buildmap_end - t_buildmap_start).count() * 1000;
    ROS_WARN("Time taken to build map: %f ms", t_buildmap);

    std::cout << "==========param==========" << std::endl;
    std::cout << "max_voxel_size:" << config.voxel_size << ", max_layer:" << config.max_layer << ", layer_size:" << config.layer_point_size.size()
                << ", max_points_size:" << config.max_points_size << ", max_cov_points_size:" << config.max_cov_points_size << ", min_eigen_value:" << config.planer_threshold << std::endl;
    std::cout << "RANSAC " << "max_iter:" << config.ransac_max_iter << ", inlier_distance_threshold:" << config.ransac_inlier_distance_threshold << ", isplane_p_threshold:" << config.ransac_isplane_p_threshold << std::endl;
    std::cout << "==========voxelmap=========" << std::endl;
    std::cout << "voxelmap size" << voxelmap.feat_map.size() << std::endl;
    std::cout << "plane size" << plane_id << std::endl;
    std::cout << "===========================" << std::endl;

    // memory log
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    double memory_usage_mb = usage.ru_maxrss / 1024.0;
    std::cout << "memory_usage_mb: " << memory_usage_mb << " MB" << std::endl;

    // publish voxel map
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud, output);
    output.header.frame_id = "camera_init";

    ros::Rate loop_rate(1);
    
    ros::Duration(1.0).sleep();
    
    pub_all();

    // publish point cloud
    while (ros::ok()) {
        point_cloud_pub.publish(output);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}