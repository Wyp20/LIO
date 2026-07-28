#pragma once
/**
 * Offline rosbag feeder for LIO nodes: call existing IMU/LiDAR callbacks
 * in timestamp order without rosbag play.
 */
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <memory>
#include <string>
#include <vector>

namespace lio_offline {

class BagFeeder {
 public:
  bool open(const std::string& bag_path, const std::string& lid_topic,
            const std::string& imu_topic) {
    try {
      bag_.open(bag_path, rosbag::bagmode::Read);
    } catch (const std::exception& e) {
      ROS_ERROR("Failed to open bag %s: %s", bag_path.c_str(), e.what());
      return false;
    }
    topics_ = {lid_topic, imu_topic};
    view_.reset(new rosbag::View(bag_, rosbag::TopicQuery(topics_)));
    it_ = view_->begin();
    end_ = view_->end();
    opened_ = true;
    ROS_INFO("Offline bag opened: %s (topics: %s, %s)", bag_path.c_str(),
             lid_topic.c_str(), imu_topic.c_str());
    return true;
  }

  bool done() const { return !opened_ || it_ == end_; }

  /** Feed until one PointCloud2 is delivered (and any IMUs before it). */
  template <typename ImuCb, typename PclCb>
  bool feedUntilLidar(ImuCb&& imu_cb, PclCb&& pcl_cb) {
    if (done()) return false;
    while (it_ != end_) {
      rosbag::MessageInstance m = *it_;
      ++it_;
      auto imu = m.instantiate<sensor_msgs::Imu>();
      if (imu) {
        imu_cb(imu);
        continue;
      }
      auto pcl = m.instantiate<sensor_msgs::PointCloud2>();
      if (pcl) {
        pcl_cb(pcl);
        return true;
      }
    }
    return false;
  }

  void close() {
    view_.reset();
    if (opened_) bag_.close();
    opened_ = false;
  }

  ~BagFeeder() { close(); }

 private:
  rosbag::Bag bag_;
  std::vector<std::string> topics_;
  std::unique_ptr<rosbag::View> view_;
  rosbag::View::iterator it_, end_;
  bool opened_ = false;
};

inline bool getBagPath(ros::NodeHandle& nh, std::string& bag_path) {
  bag_path.clear();
  // If /bag_path is explicitly set (including empty), treat it as authoritative so
  // a leftover private ~/bag_path from a prior offline run cannot force offline mode.
  if (ros::param::has("/bag_path")) {
    ros::param::get("/bag_path", bag_path);
    return !bag_path.empty();
  }
  if (nh.getParam("bag_path", bag_path) && !bag_path.empty()) return true;
  ros::NodeHandle pnh("~");
  if (pnh.getParam("bag_path", bag_path) && !bag_path.empty()) return true;
  return false;
}

}  // namespace lio_offline
