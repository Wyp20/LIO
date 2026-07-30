#include "dlio/odom.h"
#include "offline_bag_feed.hpp"

int main(int argc, char** argv) {

  mallopt(M_ARENA_MAX, 1);

  ros::init(argc, argv, "dlio_odom_node");
  ros::NodeHandle nh("~");

  dlio::OdomNode node(nh);

  std::string bag_path, lid_topic, imu_topic;
  // Remapped topics are absolute; also accept common names from parent
  nh.param<std::string>("lid_topic", lid_topic, "/lidar_points");
  nh.param<std::string>("imu_topic", imu_topic, "/fvs/imu_raw");
  // Prefer subscribed remaps: pointcloud/imu relative to private nh are remapped in launch
  if (!nh.getParam("lid_topic", lid_topic)) {
    lid_topic = "/lidar_points";
  }
  if (!nh.getParam("imu_topic", imu_topic)) {
    imu_topic = "/fvs/imu_raw";
  }

  lio_offline::BagFeeder bag_feeder;
  const bool offline_mode = lio_offline::getBagPath(nh, bag_path) &&
                            bag_feeder.open(bag_path, lid_topic, imu_topic);

  if (offline_mode) {
    node.start();
    ROS_INFO("DLIO offline bag mode (IMU lookahead 0.15s for deskew)");
    while (ros::ok()) {
      // Deskew waits for IMU stamps past scan end; feed a short IMU horizon
      // after each cloud before invoking the lidar callback.
      bool fed = bag_feeder.feedUntilLidar(
          [&](const sensor_msgs::Imu::ConstPtr& msg) { node.callbackImu(msg); },
          [&](const sensor_msgs::PointCloud2::ConstPtr& msg) {
            node.callbackPointCloud(msg);
          },
          0.15);
      if (!fed && bag_feeder.done()) break;
    }
    return 0;
  }

  ros::AsyncSpinner spinner(0);
  spinner.start();
  node.start();
  ros::waitForShutdown();

  return 0;
}
