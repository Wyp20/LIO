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

bool map_init = false;

bool downsample_enable = true;
double voxel_filter_size;
pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
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

// log related variables
double times_tamp = 0.0;
string mem_file_path, time_file_path, time_aver_file_path;
bool save_mem_flag, save_time_flag;
double time_aver = 0.0;
double time_tmp = 0.0;
int log_scan_num = 0;
std::ofstream mem_file;
std::ofstream time_file;
std::ofstream time_aver_file;

// log related functions
void save_to_log(std::ofstream &mem_file, std::ofstream &time_file) {
    if(save_mem_flag) {
        struct rusage usage;
        getrusage(RUSAGE_SELF, &usage);
        double memory_usage_mb = usage.ru_maxrss / 1024.0;
        mem_file << std::fixed << std::setprecision(4) 
                << times_tamp << " "
                << memory_usage_mb << std::endl;
        
        std::cout << "Memory usage: " << memory_usage_mb << " MB" << std::endl;
    }

    if(save_time_flag) {
        time_file << std::fixed << std::setprecision(6) 
                 << times_tamp << " "
                 << time_tmp << std::endl;
    }
}

void update_time_log() {
    log_scan_num++;
    time_aver = time_aver * (log_scan_num - 1) / log_scan_num + time_tmp / log_scan_num;
    std::cout <<"scan num: " << log_scan_num << " time_aver: " << time_aver << "ms time_current: " << time_tmp << "ms" << std::endl;
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

// service callback functions
bool publishCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    pub_all();
    return true;
}

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

// point cloud callback function
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    // 将ROS消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    times_tamp = msg->header.stamp.toSec();

    if(downsample_enable){
        voxel_filter.setInputCloud(cloud);
        voxel_filter.filter(*cloud);
    }
    ROS_INFO("Received point cloud with %ld points, after downsample %ld points", 
             cloud->points.size(), cloud->points.size());

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
    
    // convert point cloud to pointWithCov format
    std::vector<pointWithCov> pv_list;
    for (size_t i = 0; i < cloud->points.size(); i++) {
        pointWithCov pv; 
        pv.point << cloud->points[i].x, cloud->points[i].y, cloud->points[i].z;
        pv.cov = M3D::Identity() * 0.001;
        pv_list.push_back(pv);
    }

    // initialize map when first received point cloud
    if (!map_init) {
        auto t_buildmap_start = std::chrono::high_resolution_clock::now();
        
        voxelmap.build(pv_list);

        auto t_buildmap_end = std::chrono::high_resolution_clock::now();
        double t_buildmap = std::chrono::duration_cast<std::chrono::duration<double>>(t_buildmap_end - t_buildmap_start).count() * 1000;

        ROS_WARN("Time taken to build initial map: %f ms", t_buildmap);
        map_init = true;
        time_tmp = t_buildmap;
    }
    // update map after first received point cloud
    else {
        auto t_update_start = std::chrono::high_resolution_clock::now();
        voxelmap.update(pv_list);
        voxelmap.lru_cache_update();
        auto t_update_end = std::chrono::high_resolution_clock::now();
        double t_update = std::chrono::duration_cast<std::chrono::duration<double>>(t_update_end - t_update_start).count() * 1000;
        ROS_INFO("Time taken to update map: %f ms", t_update);
        
        time_tmp = t_update;
    }
    
    pub_all();

    save_to_log(mem_file, time_file);
    if(save_time_flag) {
        update_time_log();
    }

    // memory log
    struct rusage usage;
    getrusage(RUSAGE_SELF, &usage);
    double memory_usage_mb = usage.ru_maxrss / 1024.0;
    std::cout << "memory_usage_mb: " << memory_usage_mb << " MB" << std::endl;
    std::cout << "=================================" << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "voxel_map_update_test");
    ros::NodeHandle nh;

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
    nh.param<double>("downsample/voxel_size", voxel_filter_size, 0.1);
    voxel_size = config.voxel_size;
    
    nh.param<bool>("choose_range/enable", choose_range, false);
    nh.param<vector<int>>("choose_range/range_list", range_list, vector<int>());

    ros::Subscriber cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>("/cloud_registered", 1000, cloudCallback);

    // publish topic
    voxel_map_all_plane_pub = nh.advertise<visualization_msgs::MarkerArray>("all_plane", 10000);
    voxel_map_all_plane_layer_color_pub = nh.advertise<visualization_msgs::MarkerArray>("all_plane_color", 10000);
    voxel_map_all_point_pub = nh.advertise<sensor_msgs::PointCloud2>("all_point", 10000);
    voxel_map_grid_map_pub = nh.advertise<visualization_msgs::MarkerArray>("grid_map", 10000);

    // service
    ros::ServiceServer publish_again_srv = nh.advertiseService("/publish_again", publishCallback);
    ros::ServiceServer publish_all_srv = nh.advertiseService("/publish_all", publishAllCallback);

    voxelmap.init(config);

    voxel_filter.setLeafSize(voxel_filter_size, voxel_filter_size, voxel_filter_size);

    // log parameters
    nh.param<string>("mem_file_path", mem_file_path, "./mem.txt");
    nh.param<string>("time_file_path", time_file_path, "./time.txt");
    nh.param<string>("time_aver_file_path", time_aver_file_path, "./time_aver.txt");
    nh.param<bool>("save_mem_flag", save_mem_flag, false);
    nh.param<bool>("save_time_flag", save_time_flag, false);

    ROS_INFO("[log] Memory usage will be saved to %s", mem_file_path.c_str());
    ROS_INFO("[log] Time will be saved to %s", time_file_path.c_str());

    // open log files
    mem_file.open(mem_file_path, std::ios::trunc);
    time_file.open(time_file_path, std::ios::trunc);
    time_aver_file.open(time_aver_file_path, std::ios::trunc);

    if (!mem_file.is_open() || !time_file.is_open() || !time_aver_file.is_open()) {
        ROS_ERROR("Failed to open log files");
        return -1;
    }

    ros::spin();

    // close files and save average time
    mem_file.close();
    time_file.close();
    if(save_time_flag) {
        time_aver_file << std::fixed << std::setprecision(3)
                      << "map updatetime: " << time_aver << " ms" << endl
                      << "scan num: " << log_scan_num << endl;
        time_aver_file.close();
    }


    return 0;
}