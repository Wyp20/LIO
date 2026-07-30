#include <atomic>
#include <csignal>
#include <cstdio>
#include <fstream>
#include <mutex>
#include <string>
#include <unordered_map>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

namespace {

std::atomic<bool> g_stop{false};

void OnSignal(int) { g_stop = true; }

const std::unordered_map<std::string, std::string> kDefaultTopics = {
    {"faster_lio", "/cloud_registered"},
    {"fast_lio", "/cloud_registered"},
    {"point_lio", "/cloud_registered"},
    {"pv_lio", "/cloud_registered"},
    {"akf_lio", "/cloud_reg_world"},
    {"voxel_map_plus", "/cloud_registered_surf"},
    {"rvoxelmap", "/rvoxelmap/cloud_registered"},
    {"super_lio", "/lio/cloud_world"},
    {"dlio", "/robot/dlio/odom_node/pointcloud/deskewed"},
    {"bievr", "/bievr_lio/points/registered"},
};

void WriteMeta(const std::string& path, const std::string& algo, const std::string& topic,
               const std::string& cloud_dir, const std::string& merged_path, int n_msgs,
               size_t n_points) {
  std::ofstream ofs(path);
  if (!ofs) return;
  ofs << "{\n"
      << "  \"algo\": \"" << algo << "\",\n"
      << "  \"topic\": \"" << topic << "\",\n"
      << "  \"leaf_m\": 0.0,\n"
      << "  \"downsample\": false,\n"
      << "  \"every_n\": 1,\n"
      << "  \"n_msgs_seen\": " << n_msgs << ",\n"
      << "  \"n_frames_kept\": " << n_msgs << ",\n"
      << "  \"n_queue_dropped\": 0,\n"
      << "  \"n_points_raw_sum\": " << n_points << ",\n"
      << "  \"n_points_saved\": " << n_points << ",\n"
      << "  \"merged_file\": \"cloud_merged.txt\",\n"
      << "  \"merged_points\": " << n_points << ",\n"
      << "  \"cloud_dir\": \"" << cloud_dir << "\",\n"
      << "  \"merged_path\": \"" << merged_path << "\",\n"
      << "  \"format\": \"pcl_pcd_ascii_txt\"\n"
      << "}\n";
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "lio_save_clouds", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string algo, topic, out_dir, cloud_dir, stop_file;
  pnh.param<std::string>("algo", algo, "");
  pnh.param<std::string>("topic", topic, "");
  pnh.param<std::string>("out_dir", out_dir, ".");
  pnh.param<std::string>("cloud_dir", cloud_dir, "");
  pnh.param<std::string>("stop_file", stop_file, "");
  if (cloud_dir.empty()) cloud_dir = out_dir;
  if (topic.empty()) {
    auto it = kDefaultTopics.find(algo);
    if (it == kDefaultTopics.end()) {
      ROS_ERROR("Need ~topic or known ~algo");
      return 2;
    }
    topic = it->second;
  }

  // Ensure output dirs exist (best-effort).
  const std::string mkdir_cmd = "mkdir -p \"" + out_dir + "\" \"" + cloud_dir + "\"";
  if (std::system(mkdir_cmd.c_str()) != 0) {
    ROS_WARN("mkdir may have failed for out/cloud dirs");
  }

  using Cloud = pcl::PointCloud<pcl::PointXYZ>;
  Cloud::Ptr cloud(new Cloud);
  cloud->points.reserve(1 << 22);
  std::mutex mtx;
  int n_msgs = 0;

  auto cb = [&](const sensor_msgs::PointCloud2ConstPtr& msg) {
    Cloud frame;
    pcl::fromROSMsg(*msg, frame);
    if (frame.empty()) return;
    std::lock_guard<std::mutex> lk(mtx);
    cloud->points.insert(cloud->points.end(), frame.points.begin(), frame.points.end());
    ++n_msgs;
  };

  ros::Subscriber sub =
      nh.subscribe<sensor_msgs::PointCloud2>(topic, 2000, cb, ros::VoidConstPtr(),
                                             ros::TransportHints().tcpNoDelay());

  std::signal(SIGINT, OnSignal);
  std::signal(SIGTERM, OnSignal);

  ROS_INFO("lio_cloud_saver: algo=%s topic=%s cloud_dir=%s", algo.c_str(), topic.c_str(),
           cloud_dir.c_str());

  ros::Rate rate(50);
  while (ros::ok() && !g_stop) {
    if (!stop_file.empty()) {
      std::ifstream sf(stop_file);
      if (sf.good()) break;
    }
    ros::spinOnce();
    rate.sleep();
  }

  // Drain a bit for in-flight publishes.
  ros::Time t_end = ros::Time::now() + ros::Duration(1.0);
  while (ros::ok() && ros::Time::now() < t_end) {
    ros::spinOnce();
    rate.sleep();
  }
  sub.shutdown();

  size_t n_points = 0;
  {
    std::lock_guard<std::mutex> lk(mtx);
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;
    n_points = cloud->points.size();
  }

  const std::string merged_path = cloud_dir + "/cloud_merged.txt";
  ROS_INFO("Saving %zu points from %d frames -> %s (PCL ASCII)", n_points, n_msgs,
           merged_path.c_str());

  int rc = 0;
  if (n_points > 0) {
    if (pcl::io::savePCDFileASCII(merged_path, *cloud) != 0) {
      ROS_ERROR("pcl::io::savePCDFileASCII failed: %s", merged_path.c_str());
      rc = 1;
    } else {
      ROS_INFO("Saved OK: %s", merged_path.c_str());
    }
  } else {
    ROS_WARN("No points received; skip save");
  }

  WriteMeta(out_dir + "/clouds_meta.json", algo, topic, cloud_dir, merged_path, n_msgs, n_points);
  if (cloud_dir != out_dir) {
    WriteMeta(cloud_dir + "/clouds_meta.json", algo, topic, cloud_dir, merged_path, n_msgs,
              n_points);
  }

  // Also dump a short line for logs / scripts.
  std::printf(
      "{\"algo\":\"%s\",\"topic\":\"%s\",\"n_msgs_seen\":%d,\"n_points_saved\":%zu,"
      "\"merged_path\":\"%s\"}\n",
      algo.c_str(), topic.c_str(), n_msgs, n_points, merged_path.c_str());
  return rc;
}
