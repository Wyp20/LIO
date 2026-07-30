
#include <csignal>
#include <ros/ros.h>
#include "lio/super_lio.h"
#include "ros/ROSWrapper.h"
#include "offline_bag_feed.hpp"
#include "tbb_thread_limit.hpp"


using namespace LI2Sup;

void SigHandle(int sig) {
  g_flag_run = false;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "lio");
  signal(SIGINT, SigHandle);
  ros::NodeHandle nh;

  int max_num_threads = 0;
  nh.param<int>("max_num_threads", max_num_threads, 0);
  nh.param<int>("/lio/max_num_threads", max_num_threads, max_num_threads);
  auto tbb_limit = makeTbbThreadLimit(max_num_threads);
  if (max_num_threads > 0) {
    ROS_INFO("TBB parallelism limited to %d threads (max_num_threads)",
             resolveTbbMaxThreads(max_num_threads));
  }

  LoadParamFromRos(nh);

  ROSWrapper::Ptr data_wrapper = std::make_shared<ROSWrapper>();
  auto lio = std::make_shared<SuperLIO>();
  lio->setROSWrapper(data_wrapper);
  lio->init();

  std::string bag_path;
  lio_offline::BagFeeder bag_feeder;
  std::string lid = g_lidar_topic, imu = g_imu_topic;
  nh.param<std::string>("/lio/ros/lidar_topic", lid, lid);
  nh.param<std::string>("/lio/ros/imu_topic", imu, imu);

  const bool offline_mode = lio_offline::getBagPath(nh, bag_path) &&
                            bag_feeder.open(bag_path, lid, imu);

  ros::Rate rate(500);
  while (ros::ok() && g_flag_run) {
    if (offline_mode) {
      bool fed = bag_feeder.feedUntilLidar(
          [&](const sensor_msgs::Imu::ConstPtr& msg) { data_wrapper->imuHandler(msg); },
          [&](const sensor_msgs::PointCloud2::ConstPtr& msg) { data_wrapper->stdMsgHandler(msg); });
      lio->process();
      if (!fed && bag_feeder.done()) {
        for (int i = 0; i < 10; ++i) lio->process();
        break;
      }
    } else {
      data_wrapper->spinOnce();
      lio->process();
      rate.sleep();
    }
  }

  lio->saveMap();
  lio->printTimeRecord();
  return 0;
}
