#include <gflags/gflags.h>
#include <unistd.h>
#include <csignal>

#include "laser_mapping.h"
#include "offline_bag_feed.hpp"

/// Online mapping with optional offline bag_path (rosparam / ~bag_path)

DEFINE_string(traj_log_file, (std::string(std::string(ROOT_DIR) + "Log/" + "traj.txt")), "path to traj log file");
void SigHandle(int sig)
{
    akf_lio::options::FLAG_EXIT = true;
    ROS_WARN("catch sig %d", sig);
}

int main(int argc, char **argv)
{
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::InitGoogleLogging(argv[0]);
    google::ParseCommandLineFlags(&argc, &argv, true);

    ros::init(argc, argv, "akf_lio");
    ros::NodeHandle nh;

    auto laser_mapping = std::make_shared<akf_lio::LaserMapping>();
    laser_mapping->InitROS(nh);

    std::string lid_topic, imu_topic, bag_path;
    nh.param<std::string>("common/lid_topic", lid_topic, "/lidar_points");
    nh.param<std::string>("common/imu_topic", imu_topic, "/fvs/imu_raw");

    lio_offline::BagFeeder bag_feeder;
    const bool offline_mode = lio_offline::getBagPath(nh, bag_path) &&
                              bag_feeder.open(bag_path, lid_topic, imu_topic);

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);

    while (ros::ok())
    {
        if (akf_lio::options::FLAG_EXIT)
        {
            break;
        }
        if (offline_mode)
        {
            bool fed = bag_feeder.feedUntilLidar(
                [&](const sensor_msgs::Imu::ConstPtr &msg) { laser_mapping->IMUCallBack(msg); },
                [&](const sensor_msgs::PointCloud2::ConstPtr &msg) {
                    laser_mapping->StandardPCLCallBack(msg);
                });
            laser_mapping->Run();
            if (!fed && bag_feeder.done())
            {
                // drain a few more Run() calls
                for (int i = 0; i < 5; ++i) laser_mapping->Run();
                break;
            }
        }
        else
        {
            ros::spinOnce();
            laser_mapping->Run();
            rate.sleep();
        }
    }

    akf_lio::Timer::PrintAll();
    LOG(INFO) << "save trajectory to: " << FLAGS_traj_log_file;
    laser_mapping->Savetrajectory(FLAGS_traj_log_file);

    return 0;
}
