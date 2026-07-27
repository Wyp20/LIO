#ifndef VOXEL_MAP_UTIL_HPP
#define VOXEL_MAP_UTIL_HPP
#include "common_lib.h"
#include "omp.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <execution>
#include <openssl/md5.h>
#include <pcl/common/io.h>
#include <rosbag/bag.h>
#include <stdio.h>
#include <string>
#include <cstdlib>  // this header file is to support srand() function
#include <unordered_map>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <unordered_set>
#include <list>
#include <vector>
#include <iterator>

#include <sys/resource.h>

#define HASH_P 116101
#define MAX_N 10000000000

static int plane_id = 0;

std::vector<std::vector<uint8_t>> map_layer2color = {
  {255, 255, 255}, // White - layer 0
  {0, 255, 255},   // Cyan - layer 1
  {0, 255, 0},     // Green - layer 2
  {255, 0, 0},     // Red - layer 3
  {255, 255, 0},   // Yellow - layer 4 (not used)
  {255, 0, 255},   // Purple - layer 5 (not used)
  {0,100,255}      // Blue - x layer (not used)
};

// R-VoxelMap config
typedef struct voxel_map_config {
  float voxel_size;
  int max_layer;
  std::vector<int> layer_point_size;
  float planer_threshold;
  int stop_reinit_threshold;
  int reinit_threshold;
  std::vector<int> reinit_size_vec;
  int update_size_threshold;
  double update_inlier_distance_threshold;
  int max_points_size;
  int max_cov_points_size;
  int max_root_points_size;
  int lru_capacity;

  int valid_check_max_layer;
  int valid_check_min_points_size;
  int valid_check_resolution;

  int ransac_max_iter;
  int ransac_sample_num;
  double ransac_inlier_distance_threshold;
  double ransac_isplane_p_threshold;
  int sample_seed;

  std::vector<int> iter2layer_;
  int direct_match_layer;

  bool pub_cache_en;
  bool pub_grid_map_en;
  bool pub_noupdate_voxel_en;
  bool pub_effect_en;

  int pub_max_voxel_layer;
} voxel_map_config;

// a point to plane matching structure
typedef struct ptpl {
  Eigen::Vector3d normal;
  Eigen::Vector3d center;
  Eigen::Matrix<double, 6, 6> plane_cov;
  double d;
  int layer;
} ptpl;

// 3D point with covariance
typedef struct pointWithCov {
  Eigen::Vector3d point;
  Eigen::Matrix3d cov;
} pointWithCov;

typedef struct Plane {
  Eigen::Vector3d center;
  Eigen::Vector3d normal;
  Eigen::Vector3d y_normal;
  Eigen::Vector3d x_normal;
  Eigen::Matrix3d sum_ppt;
  Eigen::Matrix3d covariance;
  Eigen::Matrix<double, 6, 6> plane_cov;
  float radius = 0;
  float min_eigen_value = 1;
  float mid_eigen_value = 1;
  float max_eigen_value = 1;
  float d = 0;
  int points_size = 0;

  bool is_plane = false;
  bool is_init = false;
  int id = 0;
  // is_update and last_update_points_size are only for publish plane
  bool is_update = false;
  bool cov_need_update = false;
  int last_update_points_size = 0;
  bool update_enable = true;
} Plane;

class VOXEL_LOC {
public:
  int64_t x, y, z;

  VOXEL_LOC(int64_t vx = 0, int64_t vy = 0, int64_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC &other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
namespace std {
template <> struct hash<VOXEL_LOC> {
  int64_t operator()(const VOXEL_LOC &s) const {
    using std::hash;
    using std::size_t;
    return ((((s.z) * HASH_P) % MAX_N + (s.y)) * HASH_P) % MAX_N + (s.x);
  }
};

template<typename T1, typename T2>
struct hash<pair<T1, T2>> {
  size_t operator()(const pair<T1, T2>& p) const {
      auto h1 = hash<T1>{}(p.first);
      auto h2 = hash<T2>{}(p.second);
      return h1 ^ (h2 << 1);
  }
};
} // namespace std

typedef struct grid_map_pub_struct{
  double resolution;
  Eigen::Vector3d origin;
  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;
  std::vector<std::pair<int, int>> grid_pair_pub;
} grid_map_pub_struct;

std::vector<grid_map_pub_struct> grid_map_pub_cache;

class OctoTree {
public:
  std::list<pointWithCov> plane_points_; // all plane points in an octo tree
  std::list<pointWithCov> not_plane_points_; // all not plane points in an octo tree
  std::list<pointWithCov> new_points_;  // new points in an octo tree

  Plane *plane_ptr_;
  OctoTree *root;
  int max_layer_;
  bool indoor_mode_;
  int layer_;
  int octo_state_; // 0 is end of tree, 1 is not
  OctoTree *leaves_[8];
  double voxel_center_[3]; // x, y, z
  std::vector<int> layer_point_size_;
  float quater_length_;
  float planer_threshold_;
  int init_size_threshold_;
  int update_size_threshold_;
  
  int all_points_num_root_;
  int all_points_num_;
  int new_points_num_;

  int max_points_size_;
  int max_cov_points_size_;
  bool init_octo_;
  bool update_cov_enable_;
  bool update_enable_;
  bool root_update_enable_;

  bool stop_reinit_;
  int reinit_state_;
  
  // config
  voxel_map_config config_;
  
  OctoTree(int layer, const voxel_map_config& config)
      : layer_(layer), config_(config) {
    max_layer_ = config.max_layer;
    layer_point_size_ = config.layer_point_size;
    max_points_size_ = config.max_points_size;
    max_cov_points_size_ = config.max_cov_points_size;
    planer_threshold_ = config.planer_threshold;
    update_size_threshold_ = config.update_size_threshold;
    
    new_points_.clear();
    plane_points_.clear();
    not_plane_points_.clear();
    octo_state_ = 0;
    new_points_num_ = 0;
    all_points_num_ = 0;
    all_points_num_root_ = 0;
    init_octo_ = false;
    root_update_enable_ = true;
    update_enable_ = true;
    update_cov_enable_ = true;
    init_size_threshold_ = layer_point_size_[layer_];
    stop_reinit_ = false;
    reinit_state_ = 0;

    for (int i = 0; i < 8; i++) {
      leaves_[i] = nullptr;
    }
    plane_ptr_ = new Plane;
  }

  ~OctoTree() {
    delete plane_ptr_;
    for (int i = 0; i < 8; i++) {
      if (leaves_[i] != nullptr) {
        delete leaves_[i];
      }
    }
  }

  // using new_points_ to init plane
  void init_plane_ransac() {
    Plane *plane = plane_ptr_;
    all_points_num_ = new_points_.size();

    // It is not ideal to call srand here,
    // However, placing it at the beginning of the program still results in random outcomes, making reproduction difficult.
    srand(config_.sample_seed);
    //* RANSAC
    if(!ransac()){
      plane->is_plane = false;
      plane->is_init = true;
      return;
    }

    plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    plane->sum_ppt = Eigen::Matrix3d::Zero();
    plane->covariance = Eigen::Matrix3d::Zero();
    plane->center = Eigen::Vector3d::Zero();
    plane->normal = Eigen::Vector3d::Zero();
    plane->points_size = 0;
    plane->radius = 0;
    
    // Calculate plane parameters using the plane_points_ list
    for (const auto &pv : plane_points_) {
      plane->sum_ppt += pv.point * pv.point.transpose();
      plane->center += pv.point;
      plane->points_size++;
    }
    
    plane->center = plane->center / plane->points_size;
    plane->covariance = plane->sum_ppt / plane->points_size - plane->center * plane->center.transpose();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance);
    Eigen::Matrix3d evecs = es.eigenvectors();
    Eigen::Vector3d evals = es.eigenvalues();
    Eigen::Matrix3d::Index evalsMin, evalsMax;
    evals.rowwise().sum().minCoeff(&evalsMin);
    evals.rowwise().sum().maxCoeff(&evalsMax);
    int evalsMid = 3 - evalsMin - evalsMax;
    Eigen::Vector3d evecMin = evecs.col(evalsMin);
    Eigen::Vector3d evecMid = evecs.col(evalsMid);
    Eigen::Vector3d evecMax = evecs.col(evalsMax);
    
    //* plane valid check
    plane->normal = evecs.real().col(evalsMin);
    plane->y_normal = evecs.real().col(evalsMid);
    plane->x_normal = plane->y_normal.cross(plane->normal);

    if(layer_ <= config_.valid_check_max_layer && plane_points_.size() >= config_.valid_check_min_points_size){
      if(!plane_valid_check()){
        plane->points_size = 0;
        plane->is_init = true;
        plane->is_plane = false; 
        return;
      }
    }

    // new_points_ temporary storage of outlier points after valid check
    if(new_points_.size() > 0){
      plane->center = plane->center * plane->points_size;
      for(auto iter = new_points_.begin(); iter != new_points_.end(); iter++){
        Eigen::Vector3d point = iter->point;
        plane->sum_ppt -= point * point.transpose();
        plane->center -= point;
      }
      plane->points_size -= new_points_.size();
      plane->center = plane->center / plane->points_size;
      plane->covariance = plane->sum_ppt / plane->points_size - plane->center * plane->center.transpose();
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance);
      evecs = es.eigenvectors();
      evals = es.eigenvalues();
      evalsMin = 0;
      evalsMax = 2;
      evalsMid = 1;
      evecMin = evecs.col(evalsMin);
      evecMid = evecs.col(evalsMid);
      evecMax = evecs.col(evalsMax);

      not_plane_points_.splice(not_plane_points_.end(), new_points_);
    }

    if(evals(evalsMin) < planer_threshold_){
      // plane covariance calculation
      Eigen::Matrix3d J_Q;
      J_Q << 1.0 / plane->points_size, 0, 0, 0, 1.0 / plane->points_size, 0, 0, 0,
          1.0 / plane->points_size;
      // Calculate plane covariance using the plane_points_ list
      for (const auto &pv : plane_points_) {
        Eigen::Matrix<double, 6, 3> J;
        Eigen::Matrix3d F;
        for (int m = 0; m < 3; m++) {
          if (m != (int)evalsMin) {
            Eigen::Matrix<double, 1, 3> F_m =
                (pv.point - plane->center).transpose() /
                ((plane->points_size) * (evals[evalsMin] - evals[m])) *
                (evecs.real().col(m) * evecs.real().col(evalsMin).transpose() +
                 evecs.real().col(evalsMin) * evecs.real().col(m).transpose());
            F.row(m) = F_m;
          } else {
            Eigen::Matrix<double, 1, 3> F_m;
            F_m << 0, 0, 0;
            F.row(m) = F_m;
          }
        }
        J.block<3, 3>(0, 0) = evecs * F;
        J.block<3, 3>(3, 0) = J_Q;
        plane->plane_cov += J * pv.cov * J.transpose();
      }

      plane->normal << evecs.real()(0, evalsMin), evecs.real()(1, evalsMin),
          evecs.real()(2, evalsMin);
      plane->y_normal << evecs.real()(0, evalsMid), evecs.real()(1, evalsMid),
          evecs.real()(2, evalsMid);
      plane->x_normal = plane->y_normal.cross(plane->normal);
      plane->min_eigen_value = evals(evalsMin);
      plane->mid_eigen_value = evals(evalsMid);
      plane->max_eigen_value = evals(evalsMax);
      plane->radius = sqrt(evals(evalsMax));
      plane->d = -(plane->normal(0) * plane->center(0) +
                   plane->normal(1) * plane->center(1) +
                   plane->normal(2) * plane->center(2));
      plane->is_plane = true;
      if (plane->last_update_points_size == 0) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      } else if (plane->points_size - plane->last_update_points_size > 100) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      }

      if (!plane->is_init) {
        plane->id = plane_id;
        plane_id++;
        plane->is_init = true;
      }

    } 
    else {
      if (!plane->is_init) {
        plane->id = plane_id;
        plane_id++;
        plane->is_init = true;
      }
      if (plane->last_update_points_size == 0) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      } else if (plane->points_size - plane->last_update_points_size > 100) {
        plane->last_update_points_size = plane->points_size;
        plane->is_update = true;
      }
      not_plane_points_.splice(not_plane_points_.end(), plane_points_);
      plane->is_plane = false;
    }
  }

  void update_plane_cov(){
    Plane *plane = plane_ptr_;

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(plane->covariance);
    Eigen::Matrix3d evecs = es.eigenvectors();
    Eigen::Vector3d evals = es.eigenvalues();

    Eigen::Matrix3d::Index evalsMin, evalsMid, evalsMax;
    evals.minCoeff(&evalsMin);
    evals.maxCoeff(&evalsMax);
    evalsMid  = 3 - evalsMin - evalsMax;
    Eigen::Vector3d evecMin = evecs.real().col(evalsMin);
    Eigen::Vector3d evecMid = evecs.real().col(evalsMid);
    Eigen::Vector3d evecMax = evecs.real().col(evalsMax);


    // plane covariance calculation
    Eigen::Matrix3d J_Q;
    J_Q << 1.0 / plane->points_size, 0, 0, 
            0, 1.0 / plane->points_size, 0, 
            0, 0, 1.0 / plane->points_size;
  
    plane->plane_cov = Eigen::Matrix<double, 6, 6>::Zero();
    int plane_points_num = 0;
    for (auto &pv : plane_points_) {
      plane_points_num++;
      Eigen::Matrix<double, 6, 3> J;
      Eigen::Matrix3d F = Eigen::Matrix3d::Zero();

      const Eigen::Vector3d point_centered = pv.point - plane->center;
      for (int m = 0; m < 3; m++) {
        if (m != (int)evalsMin) {
          const Eigen::Vector3d evec_m = evecs.col(m);
          F.row(m) = 1.0 / (plane->points_size * (evals[evalsMin] - evals[m])) * point_centered.transpose() * 
                      (evec_m * evecs.col(evalsMin).transpose() + evecs.col(evalsMin) * evec_m.transpose());
        }
      }
      J.block<3, 3>(0, 0) = evecs * F;
      J.block<3, 3>(3, 0) = J_Q;
      plane->plane_cov.noalias() += J * pv.cov * J.transpose();
    }
    plane->cov_need_update = false;
    plane->is_update = true;
    return;
  }

  bool check_and_update(const pointWithCov &pv){
    Plane *plane = plane_ptr_;
    int curr_points_num = plane->points_size;
    // plane_points_num_ > plane->points_size 由于在check_update成功后会直接给plane_points_num_++，但这里要用的还是旧的plane->points_size

    Eigen::Vector3d curr_mean = plane->center;
    Eigen::Matrix3d curr_ppt = plane->sum_ppt;
    
    Eigen::Vector3d p_vec(pv.point[0], pv.point[1], pv.point[2]);
    Eigen::Vector3d new_mean = (curr_mean * curr_points_num + p_vec) / (curr_points_num + 1);
    
    // update ppt and cov
    Eigen::Matrix3d new_ppt = curr_ppt + p_vec * p_vec.transpose();
    Eigen::Matrix3d new_cov = new_ppt / (curr_points_num + 1) - new_mean * new_mean.transpose();
    
    // calculate new cov eigenvalues
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(new_cov);
    Eigen::Vector3d eigen_values = es.eigenvalues();
    
    // check if min eigenvalue is still less than threshold
    double min_eigen_value = eigen_values(0);
    if(min_eigen_value >= config_.planer_threshold){
      return false;
    }
    else{
      plane->center = new_mean;
      plane->sum_ppt = new_ppt;
      plane->covariance = new_cov;
      plane->points_size = curr_points_num + 1;

      Eigen::Matrix3d evecs = es.eigenvectors();
      plane->normal << evecs.real()(0, 0), evecs.real()(1, 0), evecs.real()(2, 0);
      plane->y_normal << evecs.real()(0, 1), evecs.real()(1, 1), evecs.real()(2, 1);
      plane->x_normal = plane->y_normal.cross(plane->normal);
    
      plane->min_eigen_value = eigen_values(0);
      plane->mid_eigen_value = eigen_values(1);
      plane->max_eigen_value = eigen_values(2);
      plane->radius = sqrt(eigen_values(2));
      plane->d = -plane->normal.dot(plane->center);
      plane->is_update = true;
      plane->cov_need_update = true;
      return true;
    }
  }

  void init_octo_tree() {
    all_points_num_ = new_points_.size();
    if (all_points_num_ > init_size_threshold_) {
      init_plane_ransac();

      if(not_plane_points_.size() > 0){
        octo_state_ = 1;
        cut_octo_tree();
      }
      else{
        octo_state_ = 0;
      }
      init_octo_ = true;
    }
    new_points_num_ = 0;
  }

  void reinit_octo_tree(){
    collect_points(new_points_, layer_);

    // test
    // for(int i = 0; i < 8; ++i){
    //   if(leaves_[i] != nullptr){
    //     leaves_[i]->check_remain_points();
    //   }
    // }

    for(int i = 0; i < 8; ++i){
      if(leaves_[i] != nullptr){
        delete leaves_[i];
        leaves_[i] = nullptr;
      }
    }

    init_octo_ = false;
    plane_ptr_->is_init = false;

    init_octo_tree();
    return;
  }

  void cut_octo_tree() {
    if (layer_ >= max_layer_) {
      octo_state_ = 0;
      return;
    }
    for (auto it = not_plane_points_.begin(); it != not_plane_points_.end();) {
      auto &pv = *it;
      int xyz[3] = {0, 0, 0};
      if (pv.point[0] > voxel_center_[0]) {
        xyz[0] = 1;
      }
      if (pv.point[1] > voxel_center_[1]) {
        xyz[1] = 1;
      }
      if (pv.point[2] > voxel_center_[2]) {
        xyz[2] = 1;
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (leaves_[leafnum] == nullptr) {
        leaves_[leafnum] = new OctoTree(layer_ + 1, config_);
        leaves_[leafnum]->root = root;
        leaves_[leafnum]->layer_point_size_ = layer_point_size_;
        leaves_[leafnum]->voxel_center_[0] =
            voxel_center_[0] + (2 * xyz[0] - 1) * quater_length_;
        leaves_[leafnum]->voxel_center_[1] =
            voxel_center_[1] + (2 * xyz[1] - 1) * quater_length_;
        leaves_[leafnum]->voxel_center_[2] =
            voxel_center_[2] + (2 * xyz[2] - 1) * quater_length_;
        leaves_[leafnum]->quater_length_ = quater_length_ / 2;
      }

      auto move_it = it++;
      leaves_[leafnum]->new_points_.splice(leaves_[leafnum]->new_points_.end(), not_plane_points_, move_it);
    }
    for (uint i = 0; i < 8; i++) {
      if (leaves_[i] != nullptr) {
        leaves_[i]->init_octo_tree();
      }
    }
  }

  void update_octo_tree() {
    if(!init_octo_){
      init_octo_tree();
      return;
    }

    // old reinit version
    // if(!stop_reinit_ && all_points_num_root_/config_.reinit_threshold > reinit_state_){
    //   reinit_octo_tree();
    //   reinit_state_ = all_points_num_root_/config_.reinit_threshold;
    //   if(all_points_num_root_ > config_.stop_reinit_threshold){
    //     stop_reinit_ = true;
    //   }
    //   return;
    // }

    // new reinit version
    if(!stop_reinit_ && all_points_num_root_> config_.reinit_size_vec[reinit_state_]){
      reinit_octo_tree();
      while(all_points_num_root_ > config_.reinit_size_vec[reinit_state_]){
        reinit_state_ += 1;
      }
      if(reinit_state_ > config_.reinit_size_vec.size() - 1){
        stop_reinit_ = true;
      }
      return;
    }

    unordered_set<OctoTree*> update_cache;
    for(auto &pv: new_points_){
        OctoTree* voxel = push_to_subvoxel(pv);
        if(voxel != nullptr){
          update_cache.insert(voxel);
        }
    }
    new_points_.clear();

    for(auto voxel : update_cache){
      voxel->update_single_voxel();
      if(voxel->plane_ptr_->is_plane && voxel->root->stop_reinit_ && voxel->plane_points_.size() > config_.max_points_size){
        voxel->update_enable_ = false;
      }
    }

    return;
  }

  void update_single_voxel(){
    if(!init_octo_){
      init_octo_tree();
      return;
    }

    if(new_points_num_ < update_size_threshold_){
      return;
    }

    if(plane_ptr_->is_plane && plane_ptr_->cov_need_update){
      update_plane_cov();
    }

    if(octo_state_ == 1 && !plane_ptr_->is_plane){
      ROS_ERROR("in update : octo_state_ == 1 && !plane_ptr_->is_plane");
    }


    if(octo_state_ == 0 && 1.0 * plane_points_.size() / (plane_points_.size() + not_plane_points_.size()) < config_.ransac_isplane_p_threshold){
      new_points_.splice(new_points_.end(), plane_points_);
      new_points_.splice(new_points_.end(), not_plane_points_);
      init_octo_tree();
    }

    new_points_num_ = 0;
  }

  void build_single_residual(const pointWithCov &pv, bool &is_sucess,
                            double &prob, ptpl &single_ptpl) {
    double radius_k = 3;
    Eigen::Vector3d p_w = pv.point;
    if (plane_ptr_->is_plane) {
      Plane &plane = *plane_ptr_;
      Eigen::Vector3d p_world_to_center = p_w - plane.center;
      double proj_x = p_world_to_center.dot(plane.x_normal);
      double proj_y = p_world_to_center.dot(plane.y_normal);
      // distance to plane: Ax+By+Cz+D 
      float dis_to_plane =
          fabs(plane.normal(0) * p_w(0) + plane.normal(1) * p_w(1) +
              plane.normal(2) * p_w(2) + plane.d);

      float dis_to_center =
          (plane.center(0) - p_w(0)) * (plane.center(0) - p_w(0)) +
          (plane.center(1) - p_w(1)) * (plane.center(1) - p_w(1)) +
          (plane.center(2) - p_w(2)) * (plane.center(2) - p_w(2));

      float range_dis = sqrt(dis_to_center - dis_to_plane * dis_to_plane);

      if (range_dis <= radius_k * plane.radius) {
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = p_w - plane.center;
        J_nq.block<1, 3>(0, 3) = -plane.normal;
        double sigma_l = J_nq * plane.plane_cov * J_nq.transpose();
        sigma_l += plane.normal.transpose() * pv.cov * plane.normal;
        if (dis_to_plane < 3.0 * sqrt(sigma_l)) {
          is_sucess = true;
          double this_prob = 1.0 / (sqrt(sigma_l)) *
                            exp(-0.5 * dis_to_plane * dis_to_plane / sigma_l);
          if (this_prob > prob) {
            prob = this_prob;
            single_ptpl.plane_cov = plane.plane_cov;
            single_ptpl.normal = plane.normal;
            single_ptpl.center = plane.center;
            single_ptpl.d = plane.d;
            single_ptpl.layer = layer_;
          }
        }
      }
    } 
    
    if (layer_ < max_layer_ && octo_state_ == 1) {
      for (size_t leafnum = 0; leafnum < 8; leafnum++) {
        if (leaves_[leafnum] != nullptr) {
          leaves_[leafnum]->build_single_residual(pv, is_sucess, prob, single_ptpl);
        }
      }
    } 
  }

  void get_point_cloud(pcl::PointCloud<pcl::PointXYZRGB> &point_cloud){

    std::vector<uint8_t> color = map_layer2color[layer_];
    if(!plane_ptr_->is_plane){
      color[0] = color[0] > 100 ? color[0] - 100 : 0;
      color[1] = color[1] > 100 ? color[1] - 100 : 0;
      color[2] = color[2] > 100 ? color[2] - 100 : 0;
    }

    if(plane_ptr_->is_plane){
      for(auto iter = plane_points_.begin(); iter != plane_points_.end(); iter++){
        pointWithCov point = *iter;
        pcl::PointXYZRGB p;
        p.x = point.point[0];
        p.y = point.point[1];
        p.z = point.point[2];
        p.r = color[0];
        p.g = color[1];
        p.b = color[2];
        point_cloud.push_back(p);
      }
    }
    else if(init_octo_){
      for(auto iter = not_plane_points_.begin(); iter != not_plane_points_.end(); iter++){
        pointWithCov point = *iter;
        pcl::PointXYZRGB p;
        p.x = point.point[0];
        p.y = point.point[1];
        p.z = point.point[2];
        p.r = color[0];
        p.g = color[1];
        p.b = color[2];
        point_cloud.push_back(p);
      }
    }
    else{
      for(auto iter = new_points_.begin(); iter != new_points_.end(); iter++){
        pointWithCov point = *iter;
        pcl::PointXYZRGB p;
        p.x = point.point[0];
        p.y = point.point[1];
        p.z = point.point[2];
        p.r = color[0];
        p.g = color[1];
        p.b = color[2];
        point_cloud.push_back(p);
      }
    }

    if(octo_state_ == 1){
      for(int i = 0; i < 8; i++){
        if(leaves_[i] != nullptr){
          leaves_[i]->get_point_cloud(point_cloud);
        }
      }
    }
  }

private:
  bool ransac(){
    Eigen::Vector3d best_normal, best_center;
    int max_inliers = 0;
    int sample_num = config_.ransac_sample_num;
    
    // create iterator vector for random access
    std::vector<std::list<pointWithCov>::iterator> iter_vec;
    iter_vec.reserve(new_points_.size());
    for(auto it = new_points_.begin(); it != new_points_.end(); ++it){
      iter_vec.push_back(it);
    }
    all_points_num_ = iter_vec.size();
    
    if(sample_num < 3){
      ROS_ERROR("ransac sample num is less than 3, set sample num to 3");
      sample_num = 3;
      config_.ransac_sample_num = sample_num;
    }
    if(sample_num > all_points_num_){
      ROS_WARN("ransac sample num is greater than all points num %d, set sample num to all points num", all_points_num_);
      sample_num = all_points_num_;
    }
    
    // RANSAC
    for(int i = 0; i < config_.ransac_max_iter; ++i){
      std::vector<Eigen::Vector3d> sample_points(sample_num);
      std::unordered_set<int> sampled_set;
      int cnt = 0;
      while(cnt < sample_num){
        int sample = rand() % all_points_num_;
        if(sampled_set.find(sample) == sampled_set.end()){
          sample_points[cnt] = iter_vec[sample]->point;
          sampled_set.insert(sample);
          cnt++;
        }
      }

      Eigen::Vector3d center = Eigen::Vector3d::Zero();
      Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
      for(int j = 0; j < sample_num; j++){
        center += sample_points[j];
        cov += sample_points[j] * sample_points[j].transpose();
      }
      center = center / sample_num;
      cov = cov / sample_num - center * center.transpose();

      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
      Eigen::Vector3d normal = es.eigenvectors().col(0);
      normal.normalize();

      int inliers = 0;
      for(int k = 0; k < all_points_num_; k++){
        Eigen::Vector3d current_point = iter_vec[k]->point;
        double distance = abs((current_point - center).dot(normal));
        if(distance < config_.ransac_inlier_distance_threshold){
          inliers++;
        }
      }

      if(inliers > max_inliers){
        max_inliers = inliers;
        best_normal = normal;
        best_center = center;
      }
    }
    
    // check if is plane
    if(1.0f * max_inliers/all_points_num_ > config_.ransac_isplane_p_threshold){
      for(int i = 0; i < all_points_num_; i++){
        auto& iter = iter_vec[i];
        double distance = abs((iter->point - best_center).dot(best_normal));
        if (distance < config_.ransac_inlier_distance_threshold) {
          plane_points_.splice(plane_points_.end(), new_points_, iter);
        } else {
          not_plane_points_.splice(not_plane_points_.end(), new_points_, iter);
        }
      }
      return true;
    }
    else{
      not_plane_points_.splice(not_plane_points_.end(), new_points_);
      return false;
    }
  }

  bool plane_valid_check() {
    Plane *plane = plane_ptr_;
    
    // project points to plane
    // !note: the valid_check_resolution must not be 0
    double resolution = quater_length_ * 4 / config_.valid_check_resolution;
    Eigen::Vector3d x_axis = plane->x_normal;
    Eigen::Vector3d y_axis = plane->y_normal;
    Eigen::Vector3d origin = plane->center;
    
    std::unordered_map<std::pair<int, int>, std::vector<std::list<pointWithCov>::iterator>> grid_map; // 位置->点云idx
    
    // iterate all points
    for (auto iter = plane_points_.begin(); iter != plane_points_.end(); iter++) {
      const Eigen::Vector3d& point = iter->point;
      
      // calculate point projection to plane
      Eigen::Vector3d relative_point = point - origin;
      double x_coord = relative_point.dot(x_axis);
      double y_coord = relative_point.dot(y_axis);

      double grid_x_f = x_coord / resolution;
      if(x_coord < 0){
        grid_x_f += -1;
      }
      double grid_y_f = y_coord / resolution;
      if(y_coord < 0){
        grid_y_f += -1;
      }

      int grid_x = (int)grid_x_f;
      int grid_y = (int)grid_y_f;

      grid_map[{grid_x, grid_y}].push_back(iter);
    }

    // for publish
    if(config_.pub_grid_map_en){
      grid_map_pub_struct tmp;
      tmp.resolution = resolution;
      tmp.origin = origin;
      tmp.x_axis = x_axis;
      tmp.y_axis = y_axis;
      for(auto grid_pair : grid_map){
        tmp.grid_pair_pub.push_back(grid_pair.first);
      }
      grid_map_pub_cache.push_back(tmp);
    }
    
    // use DFS to cluster points
    std::unordered_map<std::pair<int, int>, bool> visited;
    std::vector<std::vector<std::pair<int, int>>> clusters;
    
    // DFS
    std::function<void(int, int, std::vector<std::pair<int, int>>&)> dfs = [&](int x, int y, std::vector<std::pair<int, int>>& cluster) {
      auto key = std::make_pair(x, y);
      if (visited[key] || grid_map.find(key) == grid_map.end()) {
          return;
      }
      
      visited[key] = true;
      cluster.push_back(key);
      
      // check 4 neighbors
      dfs(x+1, y, cluster);
      dfs(x-1, y, cluster);
      dfs(x, y+1, cluster);
      dfs(x, y-1, cluster);
    };
    
    for (const auto& grid_pair : grid_map) {
      auto key = grid_pair.first;
      if (!visited[key]) {
        std::vector<std::pair<int, int>> cluster;
        dfs(key.first, key.second, cluster);
        clusters.push_back(cluster);
      }
    }

    // find the largest cluster
    int max_cluster_size = 0;
    int max_cluster_idx = -1;
    for (size_t i = 0; i < clusters.size(); i++) {
      int cluster_size = 0;
      for(auto grid_pair : clusters[i]){
        cluster_size += grid_map[grid_pair].size();
      }
      if (cluster_size > max_cluster_size) {
          max_cluster_size = cluster_size;
          max_cluster_idx = i;
      }
    }
    
    // check if the largest cluster satisfies the threshold
    if (1.0 * max_cluster_size / all_points_num_ > config_.ransac_isplane_p_threshold) {
      for(int i = 0; i < clusters.size(); i++){
        if(i == max_cluster_idx){
          continue;
        }
        for(auto grid_pair : clusters[i]){
          for(auto iter : grid_map[grid_pair]){
            new_points_.splice(new_points_.end(), plane_points_, iter);
          }
        }
      }
      return true;
    }
    else{
      not_plane_points_.splice(not_plane_points_.end(), plane_points_);
      return false;
    }
  }

  void collect_points(std::list<pointWithCov> &collect_list, int collect_layer){
    if(plane_ptr_->is_plane){
      collect_list.splice(collect_list.end(), plane_points_);
    }
    
    if(octo_state_ == 0){
      collect_list.splice(collect_list.end(), not_plane_points_);
    }

    if(!init_octo_ && layer_ != collect_layer){
      collect_list.splice(collect_list.end(), new_points_);
    }

    if(octo_state_ == 1){
      for(int i = 0; i < 8; i++){
        if(leaves_[i] != nullptr){
          leaves_[i]->collect_points(collect_list, collect_layer);
        }
      }
    }
    return;
  }

  void check_remain_points(){
    if(!plane_points_.empty() || !not_plane_points_.empty() || !new_points_.empty()){
      std::cout << "============ error =============" << std::endl;
      std::cout << "layer_: " << layer_ << std::endl;
      std::cout << "octo_state_: " << octo_state_ << std::endl;
      std::cout << "plane_ptr_->is_plane: " << plane_ptr_->is_plane << std::endl;
    }
    if(!plane_points_.empty()){
      std::cout << "plane_points_.size(): " << plane_points_.size() << std::endl;
    }
    if(!not_plane_points_.empty()){
      std::cout << "not_plane_points_.size(): " << not_plane_points_.size() << std::endl;
    }
    if(!new_points_.empty()){
      std::cout << "new_points_.size(): " << new_points_.size() << std::endl;
    }
    
    if(octo_state_ == 1){
      for(int i = 0; i < 8; i++){
        if(leaves_[i] != nullptr){
          leaves_[i]->check_remain_points();
        }
      }
    }
    return;
  }

  OctoTree* push_to_subvoxel(const pointWithCov &pv){
    vector<OctoTree *> subvoxel_candidates = find_subvoxel_candidates(pv);
    double min_residual = 1000000;
    OctoTree *best_voxel = nullptr;
    for (auto curr_voxel : subvoxel_candidates) {
      if(curr_voxel->plane_ptr_->is_plane){
        Plane *plane = curr_voxel->plane_ptr_;
        double residual = abs(plane->normal.dot(pv.point) + plane->d);
        double distance2 = (pv.point - plane->center).squaredNorm();
        double range_dis = sqrt(distance2 - residual * residual);
        if(range_dis < 3 * plane->radius){
          if(residual < 3 * sqrt(plane->min_eigen_value) && residual < min_residual){
            min_residual = residual;
            best_voxel = curr_voxel;
          }
        }
      }
    }

    if(best_voxel != nullptr && min_residual < config_.update_inlier_distance_threshold){
      if(!best_voxel->update_enable_){
        root->all_points_num_root_ -= 1;
        return nullptr;
      }
      if(best_voxel->check_and_update(pv)){
        best_voxel->plane_points_.push_back(pv);
        best_voxel->new_points_num_ ++;
        return best_voxel;
      }
    }

    auto last_voxel = subvoxel_candidates.back();
    if(last_voxel->octo_state_ == 0){
      best_voxel = last_voxel;
      if(best_voxel -> init_octo_){
        best_voxel->not_plane_points_.push_back(pv);
        best_voxel->new_points_num_ ++;
      }
      else{
        best_voxel->new_points_.push_back(pv);
      }
    }
    else{
      int xyz[3] = {0, 0, 0};
      if (pv.point[0] > last_voxel->voxel_center_[0]) {
        xyz[0] = 1;
      }
      if (pv.point[1] > last_voxel->voxel_center_[1]) {
        xyz[1] = 1;
      }
      if (pv.point[2] > last_voxel->voxel_center_[2]) {
        xyz[2] = 1;
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];

      last_voxel->leaves_[leafnum] = new OctoTree(last_voxel->layer_ + 1, last_voxel->config_);
      last_voxel->leaves_[leafnum]->root = root;
      last_voxel->leaves_[leafnum]->voxel_center_[0] =
          last_voxel->voxel_center_[0] + (2 * xyz[0] - 1) * last_voxel->quater_length_;
      last_voxel->leaves_[leafnum]->voxel_center_[1] =
          last_voxel->voxel_center_[1] + (2 * xyz[1] - 1) * last_voxel->quater_length_;
      last_voxel->leaves_[leafnum]->voxel_center_[2] =
          last_voxel->voxel_center_[2] + (2 * xyz[2] - 1) * last_voxel->quater_length_;
      last_voxel->leaves_[leafnum]->quater_length_ = last_voxel->quater_length_ / 2;

      last_voxel->leaves_[leafnum]->new_points_.push_back(pv);
      best_voxel = last_voxel->leaves_[leafnum];
    }

    return best_voxel;
  }

  vector<OctoTree *> find_subvoxel_candidates(const pointWithCov &pv) {
    vector<OctoTree *> subvoxel_candidates;
    OctoTree *curr = this;
    subvoxel_candidates.push_back(curr);
    while (curr->octo_state_ != 0) {
      int xyz[3] = {0, 0, 0};
      if (pv.point[0] > curr->voxel_center_[0]) {
        xyz[0] = 1;
      }
      if (pv.point[1] > curr->voxel_center_[1]) {
        xyz[1] = 1;
      }
      if (pv.point[2] > curr->voxel_center_[2]) {
        xyz[2] = 1;
      }
      int leafnum = 4 * xyz[0] + 2 * xyz[1] + xyz[2];
      if (curr->leaves_[leafnum] == nullptr) {
        break;
      }
      curr = curr->leaves_[leafnum];
      subvoxel_candidates.push_back(curr);
    }
    return subvoxel_candidates;
  }

  void cout_log(){
    std::cout << "[layer_ : " << layer_ ;
    std::cout << " octo_state_ : " << octo_state_ << " ]:";
    if(plane_ptr_->is_plane){
      std::cout << "is plane : " << plane_points_.size() << " not plane : " << not_plane_points_.size() << std::endl;
    }
    else if(init_octo_){
      std::cout << "not plane : " << not_plane_points_.size() << std::endl;
    }
    else{
      std::cout << "new points : " << new_points_.size() << std::endl;
    }

    if(octo_state_ == 1){
      for(int i = 0; i < 8; i++){
        if(leaves_[i] != nullptr){
          leaves_[i]->cout_log();
        }
      }
    }
  }
};

// p_imu is not used here, it can be removed directly
void transformLidar(const StatesGroup &state,
                    const PointCloudXYZI::Ptr &input_cloud,
                    pcl::PointCloud<pcl::PointXYZI>::Ptr &trans_cloud) {
  trans_cloud->clear();
  for (size_t i = 0; i < input_cloud->size(); i++) {
    pcl::PointXYZINormal p_c = input_cloud->points[i];
    Eigen::Vector3d p(p_c.x, p_c.y, p_c.z);
    // p = p_imu->Lid_rot_to_IMU * p + p_imu->Lid_offset_to_IMU;
    p = state.rot_end * p + state.pos_end;
    pcl::PointXYZI pi;
    pi.x = p(0);
    pi.y = p(1);
    pi.z = p(2);
    pi.intensity = p_c.intensity;
    trans_cloud->points.push_back(pi);
  }
}

void mapJet(double v, double vmin, double vmax, uint8_t &r, uint8_t &g,
            uint8_t &b) {
  r = 255;
  g = 255;
  b = 255;

  if (v < vmin) {
    v = vmin;
  }

  if (v > vmax) {
    v = vmax;
  }

  double dr, dg, db;

  if (v < 0.1242) {
    db = 0.504 + ((1. - 0.504) / 0.1242) * v;
    dg = dr = 0.;
  } else if (v < 0.3747) {
    db = 1.;
    dr = 0.;
    dg = (v - 0.1242) * (1. / (0.3747 - 0.1242));
  } else if (v < 0.6253) {
    db = (0.6253 - v) * (1. / (0.6253 - 0.3747));
    dg = 1.;
    dr = (v - 0.3747) * (1. / (0.6253 - 0.3747));
  } else if (v < 0.8758) {
    db = 0.;
    dr = 1.;
    dg = (0.8758 - v) * (1. / (0.8758 - 0.6253));
  } else {
    db = 0.;
    dg = 0.;
    dr = 1. - (v - 0.8758) * ((1. - 0.504) / (1. - 0.8758));
  }

  r = (uint8_t)(255 * dr);
  g = (uint8_t)(255 * dg);
  b = (uint8_t)(255 * db);
}

void CalcVectQuation(const Eigen::Vector3d &x_vec, const Eigen::Vector3d &y_vec,
                     const Eigen::Vector3d &z_vec,
                     geometry_msgs::Quaternion &q) {

  Eigen::Matrix3d rot;
  rot << x_vec(0), x_vec(1), x_vec(2), y_vec(0), y_vec(1), y_vec(2), z_vec(0),
      z_vec(1), z_vec(2);
  Eigen::Matrix3d rotation = rot.transpose();
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void CalcQuation(const Eigen::Vector3d &vec, const int axis,
                 geometry_msgs::Quaternion &q) {
  Eigen::Vector3d x_body = vec;
  Eigen::Vector3d y_body(1, 1, 0);
  if (x_body(2) != 0) {
    y_body(2) = -(y_body(0) * x_body(0) + y_body(1) * x_body(1)) / x_body(2);
  } else {
    if (x_body(1) != 0) {
      y_body(1) = -(y_body(0) * x_body(0)) / x_body(1);
    } else {
      y_body(0) = 0;
    }
  }
  y_body.normalize();
  Eigen::Vector3d z_body = x_body.cross(y_body);
  Eigen::Matrix3d rot;

  rot << x_body(0), x_body(1), x_body(2), y_body(0), y_body(1), y_body(2),
      z_body(0), z_body(1), z_body(2);
  Eigen::Matrix3d rotation = rot.transpose();
  if (axis == 2) {
    Eigen::Matrix3d rot_inc;
    rot_inc << 0, 0, 1, 0, 1, 0, -1, 0, 0;
    rotation = rotation * rot_inc;
  }
  Eigen::Quaterniond eq(rotation);
  q.w = eq.w();
  q.x = eq.x();
  q.y = eq.y();
  q.z = eq.z();
}

void calcBodyCov(Eigen::Vector3d &pb, const float range_inc,
                 const float degree_inc, Eigen::Matrix3d &cov) {
  float range = sqrt(pb[0] * pb[0] + pb[1] * pb[1] + pb[2] * pb[2]);
  float range_var = range_inc * range_inc;
  Eigen::Matrix2d direction_var;
  direction_var << pow(sin(DEG2RAD(degree_inc)), 2), 0, 0,
      pow(sin(DEG2RAD(degree_inc)), 2);
  Eigen::Vector3d direction(pb);
  direction.normalize();
  Eigen::Matrix3d direction_hat;
  direction_hat << 0, -direction(2), direction(1), direction(2), 0,
      -direction(0), -direction(1), direction(0), 0;
  Eigen::Vector3d base_vector1(1, 1,
                               -(direction(0) + direction(1)) / direction(2));
  base_vector1.normalize();
  Eigen::Vector3d base_vector2 = base_vector1.cross(direction);
  base_vector2.normalize();
  Eigen::Matrix<double, 3, 2> N;
  N << base_vector1(0), base_vector2(0), base_vector1(1), base_vector2(1),
      base_vector1(2), base_vector2(2);

  Eigen::Matrix<double, 3, 3> var;
  var.setZero();
  var(0, 0) = range_var;
  var.block(1,1,2,2) = direction_var;

  Eigen::Matrix<double, 3, 3> A;
  A.setZero();
  A.col(0) = direction;
  A.block(0,1,3,2) = - range * direction_hat * N;

  cov = (A * var * A.transpose());
};

// R-VoxelMap class
class VoxelMap {
public:
  std::unordered_map<VOXEL_LOC, OctoTree *> feat_map;
  std::unordered_map<VOXEL_LOC, std::list<VOXEL_LOC>::iterator> loc2lru;
  std::list<VOXEL_LOC> lru_cache;

  voxel_map_config config;
  float voxel_size;
  int lru_capacity;
  std::vector<int> iter2layer_;

  VoxelMap() {}

  VoxelMap(const voxel_map_config& _config) : config(_config) {}

  void init(const voxel_map_config& _config) {
    config = _config;
    voxel_size = config.voxel_size;
    lru_capacity = config.lru_capacity;
    iter2layer_ = config.iter2layer_;
  }

  ~VoxelMap() {
    for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
      delete iter->second;
    }
  }

  void build(const std::vector<pointWithCov> &input_points) {
    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
      const pointWithCov p_v = input_points[i];
      float loc_xyz[3];
      for (int j = 0; j < 3; j++) {
        loc_xyz[j] = p_v.point[j] / voxel_size;
        if (loc_xyz[j] < 0) {
          loc_xyz[j] -= 1.0;
        }
      }
      VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                         (int64_t)loc_xyz[2]);
      auto iter = feat_map.find(position);
      if (iter != feat_map.end()) {
        iter->second->new_points_.push_back(p_v);
        iter->second->all_points_num_root_ ++;
      } else {
        OctoTree *octo_tree = new OctoTree(0, config);
        feat_map[position] = octo_tree;
        feat_map[position]->root = octo_tree;
        feat_map[position]->quater_length_ = voxel_size / 4;
        feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
        feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
        feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
        feat_map[position]->new_points_.push_back(p_v);
        feat_map[position]->all_points_num_root_ ++;
        feat_map[position]->layer_point_size_ = config.layer_point_size;
      }
    }
    for (auto iter = feat_map.begin(); iter != feat_map.end(); ++iter) {
      iter->second->init_octo_tree();
      if(iter->second->all_points_num_root_ > config.max_root_points_size){
        iter->second->root_update_enable_ = false;
      }
      lru_cache.push_front(iter->first);
      loc2lru[iter->first] = lru_cache.begin();
    }
  }

  void update(const std::vector<pointWithCov> &input_points) {
    unordered_map<VOXEL_LOC, OctoTree *> update_cache;
    uint plsize = input_points.size();
    for (uint i = 0; i < plsize; i++) {
      const pointWithCov p_v = input_points[i];
      float loc_xyz[3];
      for (int j = 0; j < 3; j++) {
        loc_xyz[j] = p_v.point[j] / voxel_size;
        if (loc_xyz[j] < 0) {
          loc_xyz[j] -= 1.0;
        }
      }
      VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                         (int64_t)loc_xyz[2]);
      auto iter = feat_map.find(position);
      if (iter != feat_map.end()) {
        if(iter->second->root_update_enable_){
          iter->second->new_points_.push_back(p_v);
          iter->second->all_points_num_root_ ++;
        }
      } else {
        OctoTree *octo_tree = new OctoTree(0, config);
        feat_map[position] = octo_tree;
        feat_map[position]->root = octo_tree;
        feat_map[position]->quater_length_ = voxel_size / 4;
        feat_map[position]->voxel_center_[0] = (0.5 + position.x) * voxel_size;
        feat_map[position]->voxel_center_[1] = (0.5 + position.y) * voxel_size;
        feat_map[position]->voxel_center_[2] = (0.5 + position.z) * voxel_size;
        feat_map[position]->new_points_.push_back(p_v);
        feat_map[position]->all_points_num_root_ ++;
      }
      update_cache[position] = feat_map[position];
    }

    for (auto& pair: update_cache) {
      pair.second->update_octo_tree();

      if(pair.second->all_points_num_root_ > config.max_root_points_size){
        pair.second->root_update_enable_ = false;
      }
      // update lru cache
      if(loc2lru.find(pair.first) != loc2lru.end()){
        lru_cache.splice(lru_cache.begin(), lru_cache, loc2lru[pair.first]);
        loc2lru[pair.first] = lru_cache.begin();    
      }
      else{
        lru_cache.push_front(pair.first);
        loc2lru[pair.first] = lru_cache.begin();
      }
    }
  }

  void lru_cache_update() {
    while (lru_cache.size() > lru_capacity) {
      auto last = lru_cache.back();
      lru_cache.pop_back();

      loc2lru.erase(last);
      delete feat_map[last];
      feat_map.erase(last);
    }
  }

  void build_residual_omp(const std::vector<pointWithCov> &pv_list,
                          const int layer,
                          std::vector<ptpl> &ptpl_list,
                          std::vector<int> &useful_index,
                          std::vector<Eigen::Vector3d> &non_match) {
    double start_time = omp_get_wtime();
    // std::mutex mylock;
    ptpl_list.clear();
    std::vector<ptpl> all_ptpl_list(pv_list.size());
    std::vector<bool> useful_ptpl(pv_list.size(), false);
    #ifdef MP_EN
      omp_set_num_threads(MP_PROC_NUM);
    #pragma omp parallel for
    #endif
    for (int i = 0; i < pv_list.size(); i++) {
      pointWithCov pv = pv_list[i];
      float loc_xyz[3];
      for (int j = 0; j < 3; j++) {
        loc_xyz[j] = pv.point[j] / voxel_size;
        if (loc_xyz[j] < 0) {
          loc_xyz[j] -= 1.0;
        }
      }
      VOXEL_LOC position((int64_t)loc_xyz[0], (int64_t)loc_xyz[1],
                        (int64_t)loc_xyz[2]);
      auto iter = feat_map.find(position);
      if (iter != feat_map.end()) {
        OctoTree *current_octo = iter->second;
        ptpl single_ptpl;
        bool is_sucess = false;
        double prob = 0;
        current_octo->build_single_residual(pv, is_sucess, prob, single_ptpl);
        if (!is_sucess) {
          VOXEL_LOC near_position = position;
          if (loc_xyz[0] >
              (current_octo->voxel_center_[0] + current_octo->quater_length_)) {
            near_position.x = near_position.x + 1;
          } else if (loc_xyz[0] < (current_octo->voxel_center_[0] -
                                  current_octo->quater_length_)) {
            near_position.x = near_position.x - 1;
          }
          if (loc_xyz[1] >
              (current_octo->voxel_center_[1] + current_octo->quater_length_)) {
            near_position.y = near_position.y + 1;
          } else if (loc_xyz[1] < (current_octo->voxel_center_[1] -
                                  current_octo->quater_length_)) {
            near_position.y = near_position.y - 1;
          }
          if (loc_xyz[2] >
              (current_octo->voxel_center_[2] + current_octo->quater_length_)) {
            near_position.z = near_position.z + 1;
          } else if (loc_xyz[2] < (current_octo->voxel_center_[2] -
                                  current_octo->quater_length_)) {
            near_position.z = near_position.z - 1;
          }
          auto iter_near = feat_map.find(near_position);
          if (iter_near != feat_map.end()) {
            iter_near->second->build_single_residual(pv, is_sucess, prob, single_ptpl);
          }
        }
        if (is_sucess) {

          // mylock.lock();
          useful_ptpl[i] = true;
          all_ptpl_list[i] = single_ptpl;
          // mylock.unlock();
        } else {
          // mylock.lock();
          useful_ptpl[i] = false;
          // mylock.unlock();
        }
      }
    }

    double end_time = omp_get_wtime();
    std::cout << "build_residual_omp build residual time:" << (end_time - start_time) * 1000 << "ms" << std::endl;
    
    for (size_t i = 0; i < useful_ptpl.size(); i++) {
      if (useful_ptpl[i]) {
        ptpl_list.push_back(all_ptpl_list[i]);
        useful_index.push_back(i);
      }
      else{
        if(config.pub_effect_en){
          non_match.push_back(pv_list[i].point);
        }
      }
    }
    double end_end_time = omp_get_wtime();
    std::cout << "build_residual_omp get useful time:" << (end_end_time - end_time) * 1000 << "ms" << std::endl;
  }

  void pub_update_plane(const ros::Publisher &plane_map_pub) {
    double max_trace = 0.25;
    double pow_num = 0.2;
    ros::Rate loop(500);
    float use_alpha = 0.8;
    visualization_msgs::MarkerArray voxel_plane;
    voxel_plane.markers.reserve(1000000);
    std::vector<Plane*> pub_plane_list;
    for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++) {
      GetUpdatePlane(iter->second, pub_plane_list);
    }
    for (size_t i = 0; i < pub_plane_list.size(); i++) {
      V3D plane_cov = pub_plane_list[i]->plane_cov.block<3, 3>(0, 0).diagonal();
      double trace = plane_cov.sum();
      if (trace >= max_trace) {
        trace = max_trace;
      }
      trace = trace * (1.0 / max_trace);
      trace = pow(trace, pow_num);
      uint8_t r, g, b;
      mapJet(trace, 0, 1, r, g, b);
      Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
      double alpha;
      if (pub_plane_list[i]->is_plane) {
        alpha = use_alpha;
      } else {
        alpha = 0;
      }
      GetSinglePlaneMarker(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
    }
    plane_map_pub.publish(voxel_plane);
    loop.sleep();
  }

  int pub_all_plane(const ros::Publisher &plane_map_pub) {
    static std::vector<int> delete_plane_all;
    // delete plane
    std::cout << "=========pub_all_plane=========" << std::endl;
    visualization_msgs::MarkerArray delete_plane;
    for(int i = 0; i < delete_plane_all.size(); i++){
      visualization_msgs::Marker plane;
      plane.header.frame_id = "camera_init";
      plane.header.stamp = ros::Time();
      plane.ns = "plane";
      plane.id = delete_plane_all[i];
      plane.action = visualization_msgs::Marker::DELETE;
      delete_plane.markers.push_back(plane);
    }
    plane_map_pub.publish(delete_plane);
    delete_plane_all.clear();
    
    // pub new plane all
    int plane_size = 0;
    double max_trace = 0.25;
    double pow_num = 0.2;
    ros::Rate loop(500);
    float use_alpha = 0.8;
    visualization_msgs::MarkerArray voxel_plane;
    voxel_plane.markers.reserve(100000);
    std::vector<Plane*> pub_plane_list;
    for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++) {
      GetAllPlane(iter->second, pub_plane_list);
    }
    plane_size = pub_plane_list.size();
    for (size_t i = 0; i < pub_plane_list.size(); i++) {
      delete_plane_all.push_back(pub_plane_list[i]->id);
      V3D plane_cov = pub_plane_list[i]->plane_cov.block<3, 3>(0, 0).diagonal();
      double trace = plane_cov.sum();
      if (trace >= max_trace) {
        trace = max_trace;
      }
      trace = trace * (1.0 / max_trace);
      trace = pow(trace, pow_num);
      uint8_t r, g, b;
      mapJet(trace, 0, 1, r, g, b);
      Eigen::Vector3d plane_rgb(r / 256.0, g / 256.0, b / 256.0);
      double alpha;
    
      alpha = use_alpha;
      
      GetSinglePlaneMarker(voxel_plane, "plane", pub_plane_list[i], alpha, plane_rgb);
    }
    plane_map_pub.publish(voxel_plane);
    loop.sleep();
    std::cout << "pub_all_plane plane_size: " << plane_size << std::endl;
    return plane_size;
  }

  int pub_all_plane_layer_color(const ros::Publisher &plane_map_pub) {
    static std::vector<int> delete_plane_all;
    visualization_msgs::MarkerArray delete_plane;
    for(int i = 0; i < delete_plane_all.size(); i++){
      visualization_msgs::Marker plane;
      plane.header.frame_id = "camera_init";
      plane.header.stamp = ros::Time();
      plane.ns = "plane_color";
      plane.id = delete_plane_all[i];
      plane.action = visualization_msgs::Marker::DELETE;
      delete_plane.markers.push_back(plane);
    }
    plane_map_pub.publish(delete_plane);
    delete_plane_all.clear();

    // test
    int count = 0;
    
    // pub new plane all
    int plane_size = 0;
    double max_trace = 0.25;
    double pow_num = 0.2;
    ros::Rate loop(500);
    float use_alpha = 0.8;
    visualization_msgs::MarkerArray voxel_plane;
    voxel_plane.markers.reserve(100000);
    std::vector<std::pair<Plane*, int>> pub_plane_with_layer_list;
    for (auto iter = feat_map.begin(); iter != feat_map.end(); iter++) {
      GetAllPlane_with_layer(iter->second, pub_plane_with_layer_list);
    }
    plane_size = pub_plane_with_layer_list.size();
    for (size_t i = 0; i < pub_plane_with_layer_list.size(); i++) {
      delete_plane_all.push_back(pub_plane_with_layer_list[i].first->id);
      
      uint8_t r, g, b;
      
      Eigen::Vector3d plane_rgb(map_layer2color[pub_plane_with_layer_list[i].second][0] / 256.0,
                                map_layer2color[pub_plane_with_layer_list[i].second][1] / 256.0, 
                                map_layer2color[pub_plane_with_layer_list[i].second][2] / 256.0);
      double alpha;
    
      alpha = use_alpha;
      
      GetSinglePlaneMarker(voxel_plane, "plane_color", pub_plane_with_layer_list[i].first, alpha, plane_rgb);
    }
    plane_map_pub.publish(voxel_plane);
    loop.sleep();
    std::cout << "pub_all_plane plane_size: " << plane_size << std::endl;
    return plane_size;
  }

  void pub_all_point(const ros::Publisher &point_pub){
    sensor_msgs::PointCloud2 point_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    for(const auto &octo : feat_map){
      pcl::PointCloud<pcl::PointXYZRGB> point_cloud_tmp;
      octo.second->get_point_cloud(point_cloud_tmp);
      *cloud += point_cloud_tmp;
    }
    pcl::toROSMsg(*cloud, point_cloud);
    point_cloud.header.frame_id = "camera_init";
    point_cloud.header.stamp = ros::Time::now();
    point_pub.publish(point_cloud);
  }

  void pub_grid_map(const ros::Publisher &grid_map_pub) {
    static int marker_id = 0;
    static int axis_id = 0;

    if(grid_map_pub_cache.empty()){
      return;
    }

    visualization_msgs::MarkerArray grid_delete_markers;
    for(int i = 0; i < marker_id; i++){
      visualization_msgs::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.header.stamp = ros::Time::now();
      marker.ns = "grid_map";
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETE;
      grid_delete_markers.markers.push_back(marker);
    }
    for(int i = 0; i < axis_id; i++){
      visualization_msgs::Marker marker;
      marker.header.frame_id = "camera_init";
      marker.header.stamp = ros::Time::now();
      marker.ns = "grid_map_axis";
      marker.id = i;
      marker.action = visualization_msgs::Marker::DELETE;
      grid_delete_markers.markers.push_back(marker);
    }
    grid_map_pub.publish(grid_delete_markers);
    ros::Duration(0.01).sleep();
    marker_id = 0;
    axis_id = 0;

    visualization_msgs::MarkerArray grid_markers;
    for(int i = 0; i < grid_map_pub_cache.size(); i++){
      grid_map_pub_struct grid_map_pub_struct = grid_map_pub_cache[i];
      Eigen::Vector3d origin = grid_map_pub_struct.origin;
      Eigen::Vector3d x_axis = grid_map_pub_struct.x_axis;
      Eigen::Vector3d y_axis = grid_map_pub_struct.y_axis;
      float resolution = grid_map_pub_struct.resolution;
      
      // x_axis marker
      visualization_msgs::Marker x_axis_marker;
      x_axis_marker.header.frame_id = "camera_init";
      x_axis_marker.header.stamp = ros::Time::now();
      x_axis_marker.ns = "grid_map_axis";
      x_axis_marker.id = axis_id++;
      x_axis_marker.type = visualization_msgs::Marker::ARROW;
      x_axis_marker.action = visualization_msgs::Marker::ADD;
      
      geometry_msgs::Point start_point;
      start_point.x = origin.x();
      start_point.y = origin.y();
      start_point.z = origin.z();
      
      geometry_msgs::Point end_point;
      end_point.x = origin.x() + x_axis.x() * resolution;
      end_point.y = origin.y() + x_axis.y() * resolution;
      end_point.z = origin.z() + x_axis.z() * resolution;

      x_axis_marker.points.push_back(start_point);
      x_axis_marker.points.push_back(end_point);
      
      x_axis_marker.scale.x = 0.05;
      x_axis_marker.scale.y = 0.1;
      x_axis_marker.scale.z = 0.1;
      x_axis_marker.color.r = 1.0;
      x_axis_marker.color.g = 0.0;
      x_axis_marker.color.b = 0.0;
      x_axis_marker.color.a = 1.0;
      x_axis_marker.lifetime = ros::Duration();

      x_axis_marker.pose.orientation.x = 0.0;
      x_axis_marker.pose.orientation.y = 0.0;
      x_axis_marker.pose.orientation.z = 0.0;
      x_axis_marker.pose.orientation.w = 1.0;
      
      grid_markers.markers.push_back(x_axis_marker);
      
      // y_axis marker
      visualization_msgs::Marker y_axis_marker;
      y_axis_marker.header.frame_id = "camera_init";
      y_axis_marker.header.stamp = ros::Time::now();
      y_axis_marker.ns = "grid_map_axis";
      y_axis_marker.id = axis_id++;
      y_axis_marker.type = visualization_msgs::Marker::ARROW;
      y_axis_marker.action = visualization_msgs::Marker::ADD;
      y_axis_marker.lifetime = ros::Duration();
      
      y_axis_marker.points.push_back(start_point);
      
      geometry_msgs::Point y_end_point;
      y_end_point.x = origin.x() + y_axis.x() * resolution;
      y_end_point.y = origin.y() + y_axis.y() * resolution;
      y_end_point.z = origin.z() + y_axis.z() * resolution;
      
      y_axis_marker.points.push_back(y_end_point);
      
      y_axis_marker.scale.x = 0.05;
      y_axis_marker.scale.y = 0.1;
      y_axis_marker.scale.z = 0.1;
      y_axis_marker.color.r = 0.0;
      y_axis_marker.color.g = 1.0;
      y_axis_marker.color.b = 0.0;
      y_axis_marker.color.a = 1.0;
      y_axis_marker.lifetime = ros::Duration();

      y_axis_marker.pose.orientation.x = 0.0;
      y_axis_marker.pose.orientation.y = 0.0;
      y_axis_marker.pose.orientation.z = 0.0;
      y_axis_marker.pose.orientation.w = 1.0;

      grid_markers.markers.push_back(y_axis_marker);

      std::vector<std::pair<int, int>> &grid_pair_pub = grid_map_pub_struct.grid_pair_pub;
      
      for(int j = 0; j < grid_pair_pub.size(); j++) {
          std::pair<int, int> grid_pair = grid_pair_pub[j];
          visualization_msgs::Marker marker;
          marker.header.frame_id = "camera_init";
          marker.header.stamp = ros::Time::now();
          marker.ns = "grid_map";
          marker.id = marker_id++;
          marker.type = visualization_msgs::Marker::CUBE;
          marker.action = visualization_msgs::Marker::ADD;
          marker.lifetime = ros::Duration();
          
          int grid_x = grid_pair.first;
          int grid_y = grid_pair.second;
          
          Eigen::Vector3d point_3d = origin + 
                                  (grid_x + 0.5) * resolution * x_axis + 
                                  (grid_y + 0.5) * resolution * y_axis;
          marker.pose.position.x = point_3d.x();
          marker.pose.position.y = point_3d.y();
          marker.pose.position.z = point_3d.z();
          
          geometry_msgs::Quaternion q;
          Eigen::Vector3d normal = x_axis.cross(y_axis);
          CalcVectQuation(x_axis, y_axis, normal, q);
          marker.pose.orientation = q;
          
          marker.scale.x = resolution;
          marker.scale.y = resolution;
          marker.scale.z = 0.01;
          
          marker.color.r = static_cast<float>(rand()) / RAND_MAX;
          marker.color.g = static_cast<float>(rand()) / RAND_MAX;
          marker.color.b = static_cast<float>(rand()) / RAND_MAX;
          marker.color.a = 0.5;
          
          marker.lifetime = ros::Duration();
          grid_markers.markers.push_back(marker);
      }
    }
    
    grid_map_pub.publish(grid_markers);
    grid_map_pub_cache.clear();
  }  

private:
  void GetSinglePlaneMarker(visualization_msgs::MarkerArray &plane_pub,
                    const std::string plane_ns, const Plane *single_plane,
                    const float alpha, const Eigen::Vector3d rgb) {
    visualization_msgs::Marker plane;
    plane.header.frame_id = "camera_init";
    plane.header.stamp = ros::Time();
    plane.ns = plane_ns;
    plane.id = single_plane->id;
    plane.type = visualization_msgs::Marker::CYLINDER;
    plane.action = visualization_msgs::Marker::ADD;
    plane.pose.position.x = single_plane->center[0];
    plane.pose.position.y = single_plane->center[1];
    plane.pose.position.z = single_plane->center[2];
    geometry_msgs::Quaternion q;
    CalcVectQuation(single_plane->x_normal, single_plane->y_normal,
                    single_plane->normal, q);
    plane.pose.orientation = q;
    plane.scale.x = 3 * sqrt(single_plane->max_eigen_value);
    plane.scale.y = 3 * sqrt(single_plane->mid_eigen_value);
    plane.scale.z = 2 * sqrt(single_plane->min_eigen_value);
    plane.color.a = alpha;
    plane.color.r = rgb(0);
    plane.color.g = rgb(1);
    plane.color.b = rgb(2);
    plane.lifetime = ros::Duration();
    plane_pub.markers.push_back(plane);
  }

  void GetUpdatePlane(const OctoTree *current_octo,
                    std::vector<Plane*> &plane_list) {
    if (current_octo->layer_ > config.pub_max_voxel_layer) {
      return;
    }
    if (current_octo->plane_ptr_->is_plane && current_octo->plane_ptr_->is_update) {
      plane_list.push_back(current_octo->plane_ptr_);
      current_octo->plane_ptr_->is_update = false;
    }
    if (current_octo->layer_ < current_octo->max_layer_) {
      if (!current_octo->plane_ptr_->is_plane) {
        for (size_t i = 0; i < 8; i++) {
          if (current_octo->leaves_[i] != nullptr) {
            GetUpdatePlane(current_octo->leaves_[i], plane_list);
          }
        }
      }
    }
    return;
  }

  void GetAllPlane(const OctoTree *current_octo,
                    std::vector<Plane*> &plane_list) {
    if (current_octo->layer_ > config.pub_max_voxel_layer) {
      return;
    }
    if (current_octo->plane_ptr_->is_plane) {
      plane_list.push_back(current_octo->plane_ptr_);
    }
    if (current_octo->octo_state_ == 1) {
      for (size_t i = 0; i < 8; i++) {
        if (current_octo->leaves_[i] != nullptr) {
          GetAllPlane(current_octo->leaves_[i], plane_list);
        }
      }
    }
    return;
  }

  void GetAllPlane_with_layer(const OctoTree *current_octo,
                    std::vector<std::pair<Plane*, int>> &plane_list) {
    if (current_octo->layer_ > config.pub_max_voxel_layer) {
      return;
    }
    if (current_octo->plane_ptr_->is_plane) {
      plane_list.push_back(std::make_pair(current_octo->plane_ptr_, current_octo->layer_));
    }
    if (current_octo->octo_state_ == 1) {
      for (size_t i = 0; i < 8; i++) {
        if (current_octo->leaves_[i] != nullptr) {
          GetAllPlane_with_layer(current_octo->leaves_[i], plane_list);
        }
      }
    }
    return;
  } 

};

#endif