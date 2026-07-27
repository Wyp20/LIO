#ifndef AKF_LIO_POINTCLOUD_PREPROCESS_H
#define AKF_LIO_POINTCLOUD_PREPROCESS_H

#ifndef DISABLE_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "common_lib.h"

namespace velodyne_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        float time;
        std::uint16_t ring;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace velodyne_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                      (float, time, time)(std::uint16_t, ring, ring))
// clang-format on

namespace ouster_ros
{
    struct EIGEN_ALIGN16 Point
    {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;
        uint16_t reflectivity;
        uint8_t ring;
        uint16_t ambient;
        uint32_t range;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (float, intensity, intensity)
                                      // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                  (std::uint32_t, t, t)
                                  (std::uint16_t, reflectivity, reflectivity)
                                  (std::uint8_t, ring, ring)
                                  (std::uint16_t, ambient, ambient)
                                  (std::uint32_t, range, range)
                                  )
// clang-format on

namespace akf_lio
{

    enum class LidarType
    {
        AVIA = 1,
        VELO32,
        OUST64
    };

    /**
     * point cloud preprocess
     * just unify the point format from livox/velodyne to PCL
     */
    class PointCloudPreprocess
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        PointCloudPreprocess() = default;
        ~PointCloudPreprocess() = default;

        /// processors
    #ifndef DISABLE_LIVOX
    void Process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudType::Ptr &pcl_out);
#endif
        void Process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudType::Ptr &pcl_out);
        void Set(LidarType lid_type, double blind, int point_filter_num);

        // accessors
        double &Max_range() { return max_range_; }
        double &Blind() { return blind_; }
        int &NumScans() { return num_scans_; }
        int &PointFilterNum() { return point_filter_num_; }
        float &TimeScale() { return time_scale_; }
        LidarType GetLidarType() const { return lidar_type_; }
        void SetLidarType(LidarType lt) { lidar_type_ = lt; }
        bool deskew_ = true;

    private:
    #ifndef DISABLE_LIVOX
    void AviaHandler(const livox_ros_driver::CustomMsg::ConstPtr &msg);
#endif
        void Oust64Handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
        void VelodyneHandler(const sensor_msgs::PointCloud2::ConstPtr &msg);

        PointCloudType cloud_full_, cloud_out_;

        LidarType lidar_type_ = LidarType::AVIA;
        int point_filter_num_ = 1;
        int num_scans_ = 6;
        double blind_ = 0.01;
        double max_range_ = 500.0;
        float time_scale_ = 1e-3;
        bool given_offset_time_ = false;
    };
} // namespace akf_lio

#endif // AKF_LIO_POINTCLOUD_PREPROCESS_H
