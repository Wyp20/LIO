#include <bievr_lio/common.h>
#include <bievr_lio/log++.h>
#include <bievr_lio/synchronizer.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <tbb/global_control.h>
#include <tbb/task_arena.h>

#include <cstdlib>
#include <string>
#include <unistd.h>

#include "bievr_lio/config_loader.h"
#include "bievr_lio_ros/publisher.h"
#include "bievr_ros_common/conversions.h"
#ifdef BIEVR_WITH_LIVOX
#include <livox_ros_driver/CustomMsg.h>
#endif
#ifdef BIEVR_WITH_LIVOX2
#include <livox_ros_driver2/CustomMsg.h>
#endif

int main(int argc, char** argv) {
  srand(1);
  // ros::init strips ROS-specific arguments (remappings, __name:=, etc.) from
  // argv in place, leaving our config-file flags for loadConfigFromArgs.
  ros::init(argc, argv, "bievr_lio_bag_node");
  ros::NodeHandle nh;

  bievr::Config config;
  if (!bievr::loadConfigFromArgs({argv, argv + argc}, config)) {
    LOG(E, "Failed to load config.");
    return -1;
  }

  // Cap TBB parallelism for the whole process (0 = TBB default, i.e. all cores).
  const int n_threads =
      config.max_num_threads > 0 ? config.max_num_threads : tbb::this_task_arena::max_concurrency();
  tbb::global_control tbb_control(tbb::global_control::max_allowed_parallelism, n_threads);
  LOG(I, config.max_num_threads > 0, "TBB parallelism limited to " << n_threads << " threads.");

  std::shared_ptr<bievr::Pipeline> pipeline =
      std::make_shared<bievr::Pipeline>(config.pipeline_config);

  bievr::Synchronizer synchronizer(pipeline);

  bievr::Publisher lio_pub(nh, pipeline, "bievr_lio");
  rosbag::Bag bag;
  bag.open(config.topic_config.bag_path, rosbag::bagmode::Read);
  std::vector<std::string> topics = {config.topic_config.pointcloud_topic,
                                     config.topic_config.imu_topic};

  double start_sec = 0.0;
  double duration_sec = 0.0;
  if (const char* e = std::getenv("LIO_BAG_START_SEC")) {
    try {
      start_sec = std::stod(e);
    } catch (...) {
    }
  }
  if (const char* e = std::getenv("LIO_BAG_DURATION_SEC")) {
    try {
      duration_sec = std::stod(e);
    } catch (...) {
    }
  }
  ros::param::param("/bag_start_sec", start_sec, start_sec);
  ros::param::param("/bag_duration_sec", duration_sec, duration_sec);

  rosbag::View full(bag, rosbag::TopicQuery(topics));
  ros::Time t0 = full.getBeginTime() + ros::Duration(start_sec);
  ros::Time t1 = full.getEndTime();
  if (duration_sec > 0.0) {
    t1 = t0 + ros::Duration(duration_sec);
    if (t1 > full.getEndTime()) t1 = full.getEndTime();
  }
  LOG(I, true, "Bag window start=" << start_sec << "s duration=" << duration_sec << "s");

  rosbag::View bag_view(bag, rosbag::TopicQuery(topics), t0, t1);
  for (const rosbag::MessageInstance& msg : bag_view) {
    if (!ros::ok()) {
      break;
    }

    if ((msg.getDataType() == "sensor_msgs/PointCloud2")) {
      sensor_msgs::PointCloud2::ConstPtr s = msg.instantiate<sensor_msgs::PointCloud2>();
      bievr::StampedIntensityPointcloud pointcloud;
      bievr::msgToPointcloud(*s, pointcloud);
      synchronizer.addPointcloud(pointcloud);
      ros::spinOnce();
      if (const char* e = std::getenv("LIO_CLOUD_PACE_MS")) {
        const int ms = std::atoi(e);
        if (ms > 0) usleep(static_cast<useconds_t>(ms) * 1000);
      }
    }
#ifdef BIEVR_WITH_LIVOX
    else if ((msg.getDataType() == "livox_ros_driver/CustomMsg")) {
      livox_ros_driver::CustomMsg::ConstPtr s = msg.instantiate<livox_ros_driver::CustomMsg>();
      bievr::StampedIntensityPointcloud pointcloud;
      bievr::msgToPointcloud(*s, pointcloud);
      synchronizer.addPointcloud(pointcloud);
    }
#endif
#ifdef BIEVR_WITH_LIVOX2
    else if ((msg.getDataType() == "livox_ros_driver2/CustomMsg")) {
      livox_ros_driver2::CustomMsg::ConstPtr s = msg.instantiate<livox_ros_driver2::CustomMsg>();
      bievr::StampedIntensityPointcloud pointcloud;
      bievr::msgToPointcloud(*s, pointcloud);
      synchronizer.addPointcloud(pointcloud);
    }
#endif
    else if (msg.getDataType() == "sensor_msgs/Imu") {
      sensor_msgs::Imu::ConstPtr s = msg.instantiate<sensor_msgs::Imu>();
      bievr::ImuMeasurement imu;
      bievr::msgToImuMeasurement(*s, imu);
      synchronizer.addImu(imu);
    }
  }

  LOG(I, "Done with for loop.");
  bag.close();
  LOG(I, "Bag closed");

  return 0;
}