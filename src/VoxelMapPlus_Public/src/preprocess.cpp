#include <thread>
#include "preprocess.h"
#include <cmath>

Preprocess::Preprocess()
        : lidar_type(AVIA), blind(0.01), point_filter_num(1) {
    N_SCANS = 6;
}

Preprocess::~Preprocess() {}

// Avia input
#ifndef DISABLE_LIVOX
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg,
                         PointCloudXYZI::Ptr &pcl_surf_out) {
    avia_handler(msg);
    *pcl_surf_out = pl_surf;
}

// Other input
#endif

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg,
                         PointCloudXYZI::Ptr &pcl_out) {
    switch (lidar_type) {
        case L515:
            l515_handler(msg);
            break;
        case VELO16:
            velodyne_handler(msg);
            break;
        case HESAIxt32:
            hesai_handler(msg);
            break;
        default:
            printf("Error LiDAR Type");
            break;
    }
    *pcl_out = pl_surf;
}

// 无特征提取，把所有点都当作了面特征
#ifndef DISABLE_LIVOX
void Preprocess::avia_handler(
        const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    auto t_feature_start = std::chrono::high_resolution_clock::now();
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    int plsize = msg->point_num;
    std::vector<bool> is_valid_pt(plsize, false);

    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    for (int i = 0; i < N_SCANS; i++) {
        pl_buff[i].clear();
        pl_buff[i].reserve(plsize);
    }
    uint valid_num = 0;

    for (uint i = 1; i < plsize; i++) {
        if ((msg->points[i].line < N_SCANS) &&
            ((msg->points[i].tag & 0x30) == 0x10 ||
             (msg->points[i].tag & 0x30) == 0x00)) {
            if (i % point_filter_num == 0) {
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature =
                        msg->points[i].offset_time /
                        float(1000000); // use curvature as time of each laser points

                if (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y +
                    pl_full[i].z + pl_full[i].z < blind * blind) {
                    continue;
                }
                if(isinf(pl_full[i].x) || isinf(pl_full[i].y) || isinf(pl_full[i].z)){
                    continue;
                }

                if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7) ||
                    (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7) ||
                    (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) {
                    is_valid_pt[i] = true;
                    valid_num++;
                }
            }
        }
    }

    for (uint i = 1; i < plsize; i++) {
        if (is_valid_pt[i]) {
            pl_surf.points.push_back(pl_full[i]);
        }
    }
}

#endif

void Preprocess::l515_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pl_surf.clear();
    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    for (int i = 0; i < plsize; i++) {
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        if (i % point_filter_num == 0) {
            pl_surf.push_back(added_pt);
        }
    }
}

#define MAX_LINE_NUM 64

void Preprocess::velodyne_handler(
        const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();

    float startOri = -atan2(pl_orig.points[0].y, pl_orig.points[0].x);
    float endOri = -atan2(pl_orig.points[plsize - 1].y,
                          pl_orig.points[plsize - 1].x) +
                   2 * M_PI;
    //激光间距收束到1pi到3pi
    if (endOri - startOri > 3 * M_PI) {
        endOri -= 2 * M_PI;
    } else if (endOri - startOri < M_PI) {
        endOri += 2 * M_PI;
    }
    //过半记录标志
    bool halfPassed = false;
    for (int i = 0; i < pl_orig.size(); i++) {
        PointType added_pt;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        float angle = atan(added_pt.z / sqrt(added_pt.x * added_pt.x +
                                             added_pt.y * added_pt.y)) *
                      180 / M_PI;
        int scanID = 0;
        if (angle >= -8.83)
            scanID = int((2 - angle) * 3.0 + 0.5);
        else
            scanID = N_SCANS / 2 + int((-8.83 - angle) * 2.0 + 0.5);

        // use [0 50]  > 50 remove outlies
        if (angle > 2 || angle < -24.33 || scanID > 50 || scanID < 0) {
            continue;
        }
        float ori = -atan2(added_pt.y, added_pt.x);
        //根据扫描线是否旋转过半选择与起始位置还是终止位置进行差值计算，从而进行补偿
        if (!halfPassed) {
            //确保-pi/2 < ori - startOri < 3*pi/2
            if (ori < startOri - M_PI / 2) {
                ori += 2 * M_PI;
            } else if (ori > startOri + M_PI * 3 / 2) {
                ori -= 2 * M_PI;
            }

            if (ori - startOri > M_PI) {
                halfPassed = true;
            }
        }
            //确保-3*pi/2 < ori - endOri < pi/2
        else {
            ori += 2 * M_PI;
            if (ori < endOri - M_PI * 3 / 2) {
                ori += 2 * M_PI;
            } else if (ori > endOri + M_PI / 2) {
                ori -= 2 * M_PI;
            }
        }
        //看看旋转多少了，记录比例relTime
        // float relTime = (ori - startOri) / (endOri - startOri);
        added_pt.curvature = (ori - startOri) / (endOri - startOri) * 100.00;
        pl_surf.push_back(added_pt);
    }
}

void Preprocess::hesai_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<hesai_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    double omega_l = 3.61;
    std::vector<bool> is_first(N_SCANS, true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);
    std::vector<float> yaw_last(N_SCANS, 0.0);
    std::vector<float> time_last(N_SCANS, 0.0);

    bool given_offset_time = pl_orig.points[plsize - 1].timestamp > 0;
    double time_head = pl_orig.points[0].timestamp;

    for (int i = 0; i < plsize; i++) {
        PointType added_pt;
        added_pt.normal_x = 0;
        added_pt.normal_y = 0;
        added_pt.normal_z = 0;
        added_pt.x = pl_orig.points[i].x;
        added_pt.y = pl_orig.points[i].y;
        added_pt.z = pl_orig.points[i].z;
        added_pt.intensity = pl_orig.points[i].intensity;
        added_pt.curvature = (pl_orig.points[i].timestamp - time_head) * 1000.0;  // ms

        if (!given_offset_time) {
            int layer = pl_orig.points[i].ring;
            if (layer >= N_SCANS) continue;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

            if (is_first[layer]) {
                yaw_fp[layer] = yaw_angle;
                is_first[layer] = false;
                added_pt.curvature = 0.0;
                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
                continue;
            }

            if (yaw_angle <= yaw_fp[layer]) {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
            } else {
                added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
            }

            if (added_pt.curvature < time_last[layer])
                added_pt.curvature += 360.0 / omega_l;

            yaw_last[layer] = yaw_angle;
            time_last[layer] = added_pt.curvature;
        }

        if (i % point_filter_num == 0) {
            if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z >
                (blind * blind)) {
                pl_surf.push_back(added_pt);
            }
        }
    }
}
