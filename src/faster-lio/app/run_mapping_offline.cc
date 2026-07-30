//
// Created by xiang on 2021/10/9.
//

#include <gflags/gflags.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <unistd.h>
#include <csignal>
#include <cstdlib>
#include <yaml-cpp/yaml.h>

#include "laser_mapping.h"
#include "tbb_thread_limit.hpp"
#include "utils.h"

/// run faster-LIO in offline mode

DEFINE_string(config_file, "./config/avia.yaml", "path to config file");
DEFINE_string(bag_file, "/home/xiang/Data/dataset/fast_lio2/avia/2020-09-16-quick-shack.bag", "path to the ros bag");
DEFINE_string(time_log_file, "./Log/time.log", "path to time log file");
DEFINE_string(traj_log_file, "./Log/traj.txt", "path to traj log file");
DEFINE_double(bag_start_sec, -1.0, "skip this many seconds from bag begin (-1=env/default 0)");
DEFINE_double(bag_duration_sec, -1.0, "process this many seconds after start (-1=env/default all)");

void SigHandle(int sig) {
    faster_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

static double resolveWindow(double flag_val, const char* env_key, double def) {
    if (flag_val >= 0.0) return flag_val;
    if (const char* e = std::getenv(env_key)) {
        try {
            return std::stod(e);
        } catch (...) {
        }
    }
    return def;
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);

    // Init ROS so registered clouds can be published for eval recorders.
    ros::init(argc, argv, "faster_lio_offline");
    ros::NodeHandle nh;

    const std::string bag_file = FLAGS_bag_file;
    const std::string config_file = FLAGS_config_file;

    int max_num_threads = 0;
    try {
        auto yaml = YAML::LoadFile(config_file);
        if (yaml["max_num_threads"]) max_num_threads = yaml["max_num_threads"].as<int>();
    } catch (...) {
    }
    auto tbb_limit = makeTbbThreadLimit(max_num_threads);
    if (max_num_threads > 0) {
        LOG(INFO) << "TBB parallelism limited to " << resolveTbbMaxThreads(max_num_threads)
                  << " threads (max_num_threads)";
    }

    auto laser_mapping = std::make_shared<faster_lio::LaserMapping>();
    if (!laser_mapping->InitWithoutROS(FLAGS_config_file)) {
        LOG(ERROR) << "laser mapping init failed.";
        return -1;
    }
    // Advertise publishers (subscribers unused; we feed callbacks from bag).
    laser_mapping->SubAndPubToROS(nh);

    /// handle ctrl-c
    signal(SIGINT, SigHandle);

    const double start_sec = resolveWindow(FLAGS_bag_start_sec, "LIO_BAG_START_SEC", 0.0);
    const double duration_sec =
        resolveWindow(FLAGS_bag_duration_sec, "LIO_BAG_DURATION_SEC", 0.0);

    // just read the bag and send the data
    LOG(INFO) << "Opening rosbag, be patient";
    rosbag::Bag bag(FLAGS_bag_file, rosbag::bagmode::Read);

    rosbag::View full(bag);
    ros::Time t0 = full.getBeginTime() + ros::Duration(start_sec);
    ros::Time t1 = full.getEndTime();
    if (duration_sec > 0.0) {
        t1 = t0 + ros::Duration(duration_sec);
        if (t1 > full.getEndTime()) t1 = full.getEndTime();
    }
    LOG(INFO) << "Bag window start=" << start_sec << "s duration=" << duration_sec << "s";

    // Only feed configured lidar/imu topics (bag may contain extra PointCloud2 e.g. /rslidar_points).
    std::string lid_topic = "/lidar_points";
    std::string imu_topic = "/fvs/imu_raw";
    try {
        auto yaml = YAML::LoadFile(config_file);
        if (yaml["common"]["lid_topic"]) lid_topic = yaml["common"]["lid_topic"].as<std::string>();
        if (yaml["common"]["imu_topic"]) imu_topic = yaml["common"]["imu_topic"].as<std::string>();
    } catch (...) {
        LOG(WARNING) << "Failed to read topics from config, using defaults";
    }
    LOG(INFO) << "Filtering bag topics: lidar=" << lid_topic << " imu=" << imu_topic;

    LOG(INFO) << "Go!";
    for (const rosbag::MessageInstance &m : rosbag::View(bag, t0, t1)) {
        const std::string &topic = m.getTopic();
#ifndef DISABLE_LIVOX
        auto livox_msg = m.instantiate<livox_ros_driver::CustomMsg>();
        if (livox_msg && topic == lid_topic) {
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &livox_msg]() {
                    laser_mapping->LivoxPCLCallBack(livox_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
            ros::spinOnce();
            if (const char* e = std::getenv("LIO_CLOUD_PACE_MS")) {
                const int ms = std::atoi(e);
                if (ms > 0) usleep(static_cast<useconds_t>(ms) * 1000);
            }
            continue;
        }
#endif

        auto point_cloud_msg = m.instantiate<sensor_msgs::PointCloud2>();
        if (point_cloud_msg && topic == lid_topic) {
            faster_lio::Timer::Evaluate(
                [&laser_mapping, &point_cloud_msg]() {
                    laser_mapping->StandardPCLCallBack(point_cloud_msg);
                    laser_mapping->Run();
                },
                "Laser Mapping Single Run");
            ros::spinOnce();
            if (const char* e = std::getenv("LIO_CLOUD_PACE_MS")) {
                const int ms = std::atoi(e);
                if (ms > 0) usleep(static_cast<useconds_t>(ms) * 1000);
            }
            continue;
        }

        auto imu_msg = m.instantiate<sensor_msgs::Imu>();
        if (imu_msg && topic == imu_topic) {
            laser_mapping->IMUCallBack(imu_msg);
            continue;
        }

        if (faster_lio::options::FLAG_EXIT) {
            break;
        }
    }

    LOG(INFO) << "finishing mapping";
    laser_mapping->Finish();

    /// print the fps
    double fps = 1.0 / (faster_lio::Timer::GetMeanTime("Laser Mapping Single Run") / 1000.);
    LOG(INFO) << "Faster LIO average FPS: " << fps;

    LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    faster_lio::Timer::PrintAll();
    faster_lio::Timer::DumpIntoFile(FLAGS_time_log_file);

    return 0;
}
