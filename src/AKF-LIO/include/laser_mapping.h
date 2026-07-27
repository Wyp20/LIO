#ifndef AKF_LIO_LASER_MAPPING_H
#define AKF_LIO_LASER_MAPPING_H

#ifndef DISABLE_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif
#include <nav_msgs/Path.h>
#include <pcl/filters/voxel_grid.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <condition_variable>
#include <thread>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "imu_processing.hpp"
#include "ivox3d/ivox3d.h"
#include "options.h"
#include "pointcloud_preprocess.h"

namespace akf_lio
{

    // Constants for magic numbers
    constexpr int MAX_POINT_SELECTED_SIZE = 100000;
    constexpr int DEFAULT_TARGET_POINT_SIZE = 5000;
    constexpr double DEFAULT_GAUSSIAN_PUB_DISTANCE = 10.0;
    constexpr int DEFAULT_GAUSSIAN_PUB_MIN_COUNT = 5;
    constexpr double DEFAULT_SCAN_VOXEL_SIZE = 0.5;
    constexpr float DEFAULT_MIN_VOXEL_SIZE = 0.2;
    constexpr double DEFAULT_INITIAL_THRESHOLD = 3.0;

    // Efficient hash function for 3D grid indices - using same as ivox hash_vec<3>
    inline size_t ComputeVoxelHash(int64_t x, int64_t y, int64_t z)
    {
        // Using same hash function as ivox hash_vec<3>: Optimized Spatial Hashing for Collision Detection
        return size_t(((x) * 73856093) ^ ((y) * 471943) ^ ((z) * 83492791)) % 10000000;
    }

    class LaserMapping
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

#ifdef IVOX_NODE_TYPE_PHC
        using IVoxType = IVox<3, IVoxNodeType::PHC, PointType>;
#else
        using IVoxType = IVox<3, IVoxNodeType::DEFAULT, PointType>;
#endif

        LaserMapping();
        ~LaserMapping()
        {
            scan_undistort_ = nullptr;
            LOG(INFO) << "laser mapping deconstruct";
        }

        /// init with ros
        bool InitROS(ros::NodeHandle &nh);

        void Run();

        // callbacks of lidar and imu
        void StandardPCLCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    #ifndef DISABLE_LIVOX
    void LivoxPCLCallBack(const livox_ros_driver::CustomMsg::ConstPtr &msg);
#endif
        void IMUCallBack(const sensor_msgs::Imu::ConstPtr &msg_in);

        // sync lidar with imu
        bool SyncPackages();

        /// interface of mtk, customized obseravtion model
        void ObsModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
        ////////////////////////////// debug save / show ////////////////////////////////////////////////////////////////
        void PublishPath(const ros::Publisher pub_path);
        void PublishOdometry(const ros::Publisher &pub_odom_aft_mapped);
        void PublishFrameWorld();
        void PublishFrameRegWorld(const ros::Publisher &pub_laser_cloud_reg_world);
        void Savetrajectory(const std::string &traj_file);
        void VoxelGridDownsample(PointCloudType::Ptr &cloud_in, PointVector &cloud_down_reg);
        static bool time_list(const PointType2 &x, const PointType2 &y);
        PointType Merge2(const PointType &pt1, const PointType &pt2);

    private:
        template <typename T>
        void SetPosestamp(T &out);

        void PointBodyToWorldPub(const PointType *pi, PointType2 *const po);
        void PointBodyToWorld(PointType const *pi, PointType *const po);
        void PointBodyToWorld(const common::V3F &pi, PointType *const po);
        void PointBodyLidarToIMU(PointType const *const pi, PointType *const po);
        void PublishMap();
        void GetRainbowColor(float value, common::V3F &color);

        void MapIncremental();

        void SubAndPubToROS(ros::NodeHandle &nh);

        bool LoadParams(ros::NodeHandle &nh);

    private:
        /// modules
        IVoxType::Options ivox_options_;
        std::shared_ptr<IVoxType> ivox_ = nullptr;                   // localmap in ivox
        std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr; // point cloud preprocess
        std::shared_ptr<ImuProcess> p_imu_ = nullptr;                // imu process

        /// params
        std::vector<double> extrinT_{3, 0.0}; // lidar-imu translation
        std::vector<double> extrinR_{9, 0.0}; // lidar-imu rotation
        std::string map_file_path_;

        /// point clouds data
        CloudPtr scan_undistort_{new PointCloudType()}; // scan after undistortion

        PointVector cur_all_points;
        PointVector scan_down_reg_;
        PointVector scan_down_world_;             // downsampled scan in world
        std::vector<PointVector> nearest_points_; // nearest points of current scan
        PointVector corr_pts_;
        PointVector corr_norm_;
        PointVector map_points_;
        std::vector<float> residuals_;          // point-to-plane residuals
        std::vector<bool> point_selected_surf_; // selected points
        int point_selected_idx_[100000] = {0};
        PointVector plane_coef_; // plane coeffs

        /// ros pub and sub stuffs
        ros::Subscriber sub_pcl_;
        ros::Subscriber sub_imu_;
        ros::Publisher pub_laser_cloud_world_;
        ros::Publisher pub_laser_cloud_reg_world_;
        ros::Publisher pub_odom_aft_mapped_;
        ros::Publisher pub_path_, gt_pub_path_;
        ros::Publisher point_cov_pub;
        ros::Publisher pub_map;
        ros::Publisher map_cov_pub;

        std::mutex mtx_buffer_;
        std::deque<double> voxel_size_sliding_;
        double avg_voxel_size_;
        std::deque<double> time_buffer_;
        std::deque<PointCloudType::Ptr> lidar_buffer_;
        std::deque<sensor_msgs::Imu::ConstPtr> imu_buffer_;
        nav_msgs::Odometry odom_aft_mapped_;

        /// options
        std::ofstream fout_pre, fout_out;
        visualization_msgs::MarkerArray pa_cov;

        double last_timestamp_lidar_ = 0;
        double lidar_end_time_ = 0;
        double last_timestamp_imu_ = -1.0;
        double first_lidar_time_ = 0.0;
        bool lidar_pushed_ = false;
        double init_residual_ = 0.0;
        double t_ratio_b_ = 1;
        double t_stop_pseudo_merge_ = 1;

        /// statistics and flags ///
        int scan_count_ = 0;
        int publish_count_ = 0;
        bool flg_first_scan_ = true;
        bool flg_EKF_inited_ = false;
        int pcd_index_ = 0;
        double lidar_mean_scantime_ = 0.0;
        int scan_num_ = 0;
        bool timediff_set_flg_ = false;
        int effect_feat_num_ = 0, frame_num_ = 0;

        ///////////////////////// EKF inputs and output ///////////////////////////////////////////////////////
        common::MeasureGroup measures_;                   // sync IMU and lidar scan
        esekfom::esekf<state_ikfom, 12, input_ikfom> kf_; // esekf
        state_ikfom state_point_;                         // ekf current state
        Eigen::Matrix<double, 6, 6> state_cov_, pred_cov_;
        vect3 pos_lidar_;                             // lidar position after eskf update
        common::V3D euler_cur_ = common::V3D::Zero(); // rotation in euler angles

        /////////////////////////  debug show / save /////////////////////////////////////////////////////////
        bool run_in_offline_ = false;
        bool path_pub_en_ = true;
        bool scan_reg_pub_en_ = false;
        bool dense_pub_en_ = false;
        bool map_pub_en_ = false;
        bool gaussian_publish_en_ = false;
        double gaussian_pub_dis_ = 10;
        double gaussian_pub_min_cnt_ = 5;
        bool adap_voxel_size_en_ = true;

        bool pcd_save_en_ = false;
        bool runtime_pos_log_ = true;
        int pcd_save_interval_ = -1;
        std::string dataset_;
        int target_point_size_ = 5000;
        double scan_voxel_size_ = 0.5;
        float min_voxel_size_ = 0.2;
        int cur_iter = 0;

        PointCloudType::Ptr pcl_wait_save_{new PointCloudType()}; // debug save
        nav_msgs::Path path_, gt_path_;
        geometry_msgs::PoseStamped msg_body_pose_;

        double init_uncertainty_ = 0.01;
    };

} // namespace akf_lio

#endif // AKF_LIO_LASER_MAPPING_H