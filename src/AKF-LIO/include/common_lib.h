#ifndef COMMON_LIB_H
#define COMMON_LIB_H

#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <boost/array.hpp>
#include <unsupported/Eigen/ArpackSupport>

#include "akf_lio/Pose6D.h"
#include "options.h"
#include "so3_math.h"

typedef struct pointWithIDTime
{
    pointWithIDTime() : x(0),
                        y(0),
                        z(0),
                        intensity(0),
                        pt_num(1),
                        use_num(1),
                        cov(Eigen::Matrix3d::Zero()),
                        time(0),
                        uncertainty(0.0)
    {
    }

    double x, y, z, intensity;
    int pt_num, use_num;
    Eigen::Matrix3d cov;
    double time;
    double uncertainty; // like the scale factor for ellipsoid
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} PointType;

using PointType2 = pcl::PointXYZINormal;
using PointCloudType = pcl::PointCloud<PointType2>;
using CloudPtr = PointCloudType::Ptr;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

namespace akf_lio::common
{

    constexpr double G_m_s2 = 9.81; // Gravity const in GuangDong/China

    template <typename S>
    inline Eigen::Matrix<S, 3, 1> VecFromArray(const std::vector<double> &v)
    {
        return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
    }

    template <typename S>
    inline Eigen::Matrix<S, 3, 1> VecFromArray(const boost::array<S, 3> &v)
    {
        return Eigen::Matrix<S, 3, 1>(v[0], v[1], v[2]);
    }

    template <typename S>
    inline Eigen::Matrix<S, 3, 3> MatFromArray(const std::vector<double> &v)
    {
        Eigen::Matrix<S, 3, 3> m;
        m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
        return m;
    }

    template <typename S>
    inline Eigen::Matrix<S, 3, 3> MatFromArray(const boost::array<S, 9> &v)
    {
        Eigen::Matrix<S, 3, 3> m;
        m << v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8];
        return m;
    }

    inline std::string DEBUG_FILE_DIR(const std::string &name) { return std::string(ROOT_DIR) + "Log/" + name; }

    using Pose6D = akf_lio::Pose6D;
    using V3D = Eigen::Vector3d;
    using V4D = Eigen::Vector4d;
    using V5D = Eigen::Matrix<double, 5, 1>;
    using M3D = Eigen::Matrix3d;
    using M4D = Eigen::Matrix4d;
    using V3F = Eigen::Vector3f;
    using V4F = Eigen::Vector4f;
    using V5F = Eigen::Matrix<float, 5, 1>;
    using M3F = Eigen::Matrix3f;
    using M4F = Eigen::Matrix4f;

    using VV3D = std::vector<V3D, Eigen::aligned_allocator<V3D>>;
    using VV3F = std::vector<V3F, Eigen::aligned_allocator<V3F>>;
    using VV4F = std::vector<V4F, Eigen::aligned_allocator<V4F>>;
    using VV4D = std::vector<V4D, Eigen::aligned_allocator<V4D>>;
    using VV5F = std::vector<V5F, Eigen::aligned_allocator<V5F>>;
    using VV5D = std::vector<V5D, Eigen::aligned_allocator<V5D>>;

    const M3D Eye3d = M3D::Identity();
    const M3F Eye3f = M3F::Identity();
    const V3D Zero3d(0, 0, 0);
    const V3F Zero3f(0, 0, 0);

    /// sync imu and lidar measurements
    struct MeasureGroup
    {
        MeasureGroup() { this->lidar_.reset(new PointCloudType()); };

        double lidar_bag_time_ = 0;
        double lidar_end_time_ = 0;
        PointCloudType::Ptr lidar_ = nullptr;
        std::deque<sensor_msgs::Imu::ConstPtr> imu_;
    };

    template <typename T>
    T rad2deg(const T &radians)
    {
        return radians * 180.0 / M_PI;
    }

    template <typename T>
    T deg2rad(const T &degrees)
    {
        return degrees * M_PI / 180.0;
    }

    /**
     * set a pose 6d from ekf status
     * @tparam T
     * @param t
     * @param a
     * @param g
     * @param v
     * @param p
     * @param R
     * @return
     */
    template <typename T>
    Pose6D set_pose6d(const double t, const Eigen::Matrix<T, 3, 1> &a, const Eigen::Matrix<T, 3, 1> &g,
                      const Eigen::Matrix<T, 3, 1> &v, const Eigen::Matrix<T, 3, 1> &p, const Eigen::Matrix<T, 3, 3> &R)
    {
        Pose6D rot_kp;
        rot_kp.offset_time = t;
        for (int i = 0; i < 3; i++)
        {
            rot_kp.acc[i] = a(i);
            rot_kp.gyr[i] = g(i);
            rot_kp.vel[i] = v(i);
            rot_kp.pos[i] = p(i);
            for (int j = 0; j < 3; j++)
                rot_kp.rot[i * 3 + j] = R(i, j);
        }
        return rot_kp;
    }

    /* comment
    plane equation: Ax + By + Cz + D = 0
    convert to: A/D*x + B/D*y + C/D*z = -1
    solve: A0*x0 = b0
    where A0_i = [x_i, y_i, z_i], x0 = [A/D, B/D, C/D]^T, b0 = [-1, ..., -1]^T
    normvec_:  normalized x0
    */
    /**
     * Calculate normal vector for a set of points
     * @tparam T
     * @param normvec
     * @param point
     * @param threshold
     * @param point_num
     * @return
     */
    template <typename T>
    bool esti_normvector(Eigen::Matrix<T, 3, 1> &normvec, const PointVector &point, const T &threshold,
                         const int &point_num)
    {
        Eigen::MatrixXf A(point_num, 3);
        Eigen::MatrixXf b(point_num, 1);
        b.setOnes();
        b *= -1.0f;

        for (int j = 0; j < point_num; j++)
        {
            A(j, 0) = point[j].x;
            A(j, 1) = point[j].y;
            A(j, 2) = point[j].z;
        }
        normvec = A.colPivHouseholderQr().solve(b);

        for (int j = 0; j < point_num; j++)
        {
            if (fabs(normvec(0) * point[j].x + normvec(1) * point[j].y + normvec(2) * point[j].z + 1.0f) > threshold)
            {
                return false;
            }
        }

        normvec.normalize();
        return true;
    }

    /**
     * squared distance
     * @param p1
     * @param p2
     * @return
     */
    inline float CalcDistance(const PointType &p1, const PointType &p2)
    {
        return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z);
    }

    inline float CalcDistance(const Eigen::Vector3f &p1, const Eigen::Vector3f &p2) { return (p1 - p2).squaredNorm(); }

} // namespace akf_lio::common
#endif