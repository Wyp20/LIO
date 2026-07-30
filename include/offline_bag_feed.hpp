#pragma once
/**
 * Offline rosbag feeder for LIO nodes: call existing IMU/LiDAR callbacks
 * in timestamp order without rosbag play.
 *
 * Optional window via ROS params / env (bag-relative seconds from first msg):
 *   /bag_start_sec, /bag_duration_sec
 *   LIO_BAG_START_SEC, LIO_BAG_DURATION_SEC
 */
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

#include <cstdlib>
#include <memory>
#include <string>
#include <vector>
#include <unistd.h>

namespace lio_offline {

inline double envOrParamDouble(const char* param, const char* env, double def = 0.0) {
  double v = def;
  if (ros::param::get(param, v)) return v;
  if (const char* e = std::getenv(env)) {
    try {
      return std::stod(e);
    } catch (...) {
    }
  }
  return def;
}

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

    const double start_sec = envOrParamDouble("/bag_start_sec", "LIO_BAG_START_SEC", 0.0);
    const double duration_sec =
        envOrParamDouble("/bag_duration_sec", "LIO_BAG_DURATION_SEC", 0.0);

    rosbag::View full(bag_, rosbag::TopicQuery(topics_));
    if (full.begin() == full.end()) {
      ROS_ERROR("Offline bag has no messages on topics %s / %s", lid_topic.c_str(),
                imu_topic.c_str());
      return false;
    }
    const ros::Time t0 = full.getBeginTime() + ros::Duration(start_sec);
    ros::Time t1 = full.getEndTime();
    if (duration_sec > 0.0) {
      t1 = t0 + ros::Duration(duration_sec);
      if (t1 > full.getEndTime()) t1 = full.getEndTime();
    }
    view_.reset(new rosbag::View(bag_, rosbag::TopicQuery(topics_), t0, t1));
    it_ = view_->begin();
    end_ = view_->end();
    opened_ = true;
    ROS_INFO(
        "Offline bag opened: %s (topics: %s, %s) window start=%.3fs duration=%.3fs "
        "msgs=%u",
        bag_path.c_str(), lid_topic.c_str(), imu_topic.c_str(), start_sec, duration_sec,
        view_->size());
    return true;
  }

  bool done() const {
    return !opened_ || (it_ == end_ && !pending_pcl_);
  }

  /**
   * Feed until one PointCloud2 is delivered (and any IMUs before it).
   * If imu_horizon_sec > 0, also feed IMUs after the cloud up to
   * cloud_stamp + horizon (needed by DLIO deskew which waits for post-scan IMU).
   * The next cloud, if encountered while looking ahead, is buffered.
   */
  template <typename ImuCb, typename PclCb>
  bool feedUntilLidar(ImuCb&& imu_cb, PclCb&& pcl_cb, double imu_horizon_sec = 0.0) {
    if (done()) return false;

    sensor_msgs::PointCloud2::ConstPtr pcl_msg;
    if (pending_pcl_) {
      pcl_msg = pending_pcl_;
      pending_pcl_.reset();
    } else {
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
          pcl_msg = pcl;
          break;
        }
      }
      if (!pcl_msg) return false;
    }

    if (imu_horizon_sec > 0.0) {
      const double horizon = pcl_msg->header.stamp.toSec() + imu_horizon_sec;
      while (it_ != end_) {
        rosbag::MessageInstance m = *it_;
        auto imu = m.instantiate<sensor_msgs::Imu>();
        if (imu) {
          if (imu->header.stamp.toSec() > horizon) break;
          ++it_;
          imu_cb(imu);
          continue;
        }
        auto pcl = m.instantiate<sensor_msgs::PointCloud2>();
        if (pcl) {
          // Keep next cloud for subsequent call; stop IMU lookahead.
          pending_pcl_ = pcl;
          ++it_;
          break;
        }
        ++it_;
      }
    }

    pcl_cb(pcl_msg);
    // Slow offline publish so external cloud savers can keep up (dense ascii dump).
    if (const char* e = std::getenv("LIO_CLOUD_PACE_MS")) {
      const int ms = std::atoi(e);
      if (ms > 0) {
        ros::spinOnce();
        usleep(static_cast<useconds_t>(ms) * 1000);
      }
    }
    return true;
  }

  void close() {
    view_.reset();
    pending_pcl_.reset();
    if (opened_) bag_.close();
    opened_ = false;
  }

  ~BagFeeder() { close(); }

 private:
  rosbag::Bag bag_;
  std::vector<std::string> topics_;
  std::unique_ptr<rosbag::View> view_;
  rosbag::View::iterator it_, end_;
  sensor_msgs::PointCloud2::ConstPtr pending_pcl_;
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
