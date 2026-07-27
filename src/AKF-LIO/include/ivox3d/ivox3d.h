#ifndef AKF_LIO_IVOX3D_H
#define AKF_LIO_IVOX3D_H

#include <glog/logging.h>
#include <execution>
#include <list>
#include <thread>

#include "eigen_types.h"
#include "ivox3d_node.hpp"

namespace akf_lio
{

    enum class IVoxNodeType
    {
        DEFAULT, // linear ivox
        PHC,     // phc ivox
    };

    /// traits for NodeType
    template <IVoxNodeType node_type, typename PointT, int dim>
    struct IVoxNodeTypeTraits
    {
    };

    template <typename PointT, int dim>
    struct IVoxNodeTypeTraits<IVoxNodeType::DEFAULT, PointT, dim>
    {
        using NodeType = IVoxNode<PointT, dim>;
    };

    template <typename PointT, int dim>
    struct IVoxNodeTypeTraits<IVoxNodeType::PHC, PointT, dim>
    {
        using NodeType = IVoxNodePhc<PointT, dim>;
    };

    template <int dim = 3, IVoxNodeType node_type = IVoxNodeType::DEFAULT, typename PointType = pcl::PointXYZ>
    class IVox
    {
    public:
        using KeyType = Eigen::Matrix<int, dim, 1>;
        using PtType = Eigen::Matrix<double, dim, 1>;
        using NodeType = typename IVoxNodeTypeTraits<node_type, PointType, dim>::NodeType;
        using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;
        using DistPoint = typename NodeType::DistPoint;

        enum class NearbyType
        {
            CENTER, // center only
            NEARBY6,
            NEARBY18,
            NEARBY26,
        };

        struct Options
        {
            double resolution_ = 0.5;                      // ivox resolution
            double inv_resolution_ = 10.0;                 // inverse resolution
            NearbyType nearby_type_ = NearbyType::NEARBY6; // nearby range
            std::size_t capacity_ = 10000000;              // capacity
        };

        /**
         * constructor
         * @param options  ivox options
         */
        explicit IVox(Options options) : options_(options)
        {
            options_.inv_resolution_ = 1.0 / options_.resolution_;
            GenerateNearbyGrids();
        }

        /**
         * add points
         * @param points_to_add
         */
        void AddPoints(const PointVector &points_to_add);

        void ErasePoints(Eigen::Vector3d pose, double cur_time);

        void UpdateUncertainty(const PointVector &points_to_add);

        /// get nn
        bool GetClosestPoint(const PointType &pt, PointType &closest_pt);

        /// get nn with condition
        bool GetClosestPoint(const PointType &pt, PointVector &closest_pt, int max_num = 1, double max_range = INFINITY);

        /// get nn in cloud
        bool GetClosestPoint(const PointVector &cloud, PointVector &closest_cloud);

        /// get number of points
        size_t NumPoints() const;

        /// get number of valid grids
        size_t NumValidGrids() const;

        /// get statistics of the points

        void GetMapPoints(PointVector &map_points);

    private:
        /// generate the nearby grids according to the given options
        void GenerateNearbyGrids();

        /// position to grid
        KeyType Pos2Grid(const PtType &pt) const;

        Options options_;
        std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator, hash_vec<dim>>
            grids_map_;                                       // voxel hash map
        std::list<std::pair<KeyType, NodeType>> grids_cache_; // voxel cache
        std::vector<KeyType> nearby_grids_;                   // nearbys
    };

    template <int dim, IVoxNodeType node_type, typename PointType>
    bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType &pt, PointType &closest_pt)
    {
        std::vector<DistPoint> candidates;
        auto key = Pos2Grid(ToEigen<double, dim>(pt));
        std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &candidates, &pt, this](const KeyType &delta)
                      {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            DistPoint dist_point;
            bool found = iter->second->second.NNPoint(pt, dist_point);
            if (found) {
                candidates.emplace_back(dist_point);
            }
        } });

        if (candidates.empty())
        {
            return false;
        }

        auto iter = std::min_element(candidates.begin(), candidates.end());
        closest_pt = iter->Get();
        return true;
    }

    template <int dim, IVoxNodeType node_type, typename PointType>
    bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType &pt, PointVector &closest_pt, int max_num,
                                                          double max_range)
    {
        std::vector<DistPoint> candidates;
        candidates.reserve(max_num * nearby_grids_.size());

        auto key = Pos2Grid(ToEigen<double, dim>(pt));

        for (const KeyType &delta : nearby_grids_)
        {
            auto dkey = key + delta;
            auto iter = grids_map_.find(dkey);
            if (iter != grids_map_.end())
            {
                auto tmp = iter->second->second.KNNPointByCondition(candidates, pt, max_num, max_range);
            }
        }
        //
        if (candidates.empty())
        {
            return false;
        }
        if (candidates.size() <= max_num)
        {
        }
        else
        {
            std::nth_element(candidates.begin(), candidates.begin() + max_num - 1, candidates.end());
            candidates.resize(max_num);
        }
        std::sort(candidates.begin(), candidates.end());
        closest_pt.clear();
        for (auto &it : candidates)
        {
            closest_pt.emplace_back(it.Get());
        }
        return closest_pt.empty() == false;
    }

    template <int dim, IVoxNodeType node_type, typename PointType>
    size_t IVox<dim, node_type, PointType>::NumValidGrids() const
    {
        return grids_map_.size();
    }

    template <int dim, IVoxNodeType node_type, typename PointType>
    void IVox<dim, node_type, PointType>::GenerateNearbyGrids()
    {
        if (options_.nearby_type_ == NearbyType::CENTER)
        {
            nearby_grids_.emplace_back(KeyType::Zero());
        }
        else if (options_.nearby_type_ == NearbyType::NEARBY6)
        {
            nearby_grids_ = {KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                             KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1)};
        }
        else if (options_.nearby_type_ == NearbyType::NEARBY18)
        {
            nearby_grids_ = {KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                             KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1), KeyType(1, 1, 0),
                             KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                             KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                             KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1)};
        }
        else if (options_.nearby_type_ == NearbyType::NEARBY26)
        {
            nearby_grids_ = {KeyType(0, 0, 0), KeyType(-1, 0, 0), KeyType(1, 0, 0), KeyType(0, 1, 0),
                             KeyType(0, -1, 0), KeyType(0, 0, -1), KeyType(0, 0, 1), KeyType(1, 1, 0),
                             KeyType(-1, 1, 0), KeyType(1, -1, 0), KeyType(-1, -1, 0), KeyType(1, 0, 1),
                             KeyType(-1, 0, 1), KeyType(1, 0, -1), KeyType(-1, 0, -1), KeyType(0, 1, 1),
                             KeyType(0, -1, 1), KeyType(0, 1, -1), KeyType(0, -1, -1), KeyType(1, 1, 1),
                             KeyType(-1, 1, 1), KeyType(1, -1, 1), KeyType(1, 1, -1), KeyType(-1, -1, 1),
                             KeyType(-1, 1, -1), KeyType(1, -1, -1), KeyType(-1, -1, -1)};
        }
        else
        {
            LOG(ERROR) << "Unknown nearby_type!";
        }
    }

    template <int dim, IVoxNodeType node_type, typename PointType>
    bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointVector &cloud, PointVector &closest_cloud)
    {
        std::vector<size_t> index(cloud.size());
        for (int i = 0; i < cloud.size(); ++i)
        {
            index[i] = i;
        }
        closest_cloud.resize(cloud.size());

        std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&cloud, &closest_cloud, this](size_t idx)
                      {
        PointType pt;
        if (GetClosestPoint(cloud[idx], pt)) {
            closest_cloud[idx] = pt;
        } else {
            closest_cloud[idx] = PointType();
        } });
        return true;
    }

    template <int dim, IVoxNodeType node_type, typename PointType>
    void IVox<dim, node_type, PointType>::ErasePoints(Eigen::Vector3d pose, double cur_time)
    {
        // LRU REMOVE OLD VOXEL
        size_t max_capacity_ = double(options_.capacity_) / options_.resolution_ / options_.resolution_;
        while (!grids_map_.empty())
        {
            if (!grids_cache_.back().second.points_.empty())
            {
                if (cur_time - grids_cache_.back().second.points_.back().time < options::TIME_TO_DELETE_LOCAL_MAP)
                    break;
            }
            grids_map_.erase(grids_cache_.back().first);
            grids_cache_.pop_back();
        }

        while (grids_map_.size() >= max_capacity_)
        {
            grids_map_.erase(grids_cache_.back().first);
            grids_cache_.pop_back();
        }
    }

    // multiple thread fast but not accurate knn
    template <int dim, IVoxNodeType node_type, typename PointType>
    void IVox<dim, node_type, PointType>::UpdateUncertainty(const PointVector &points_to_add)
    {
        int add_pts_size = points_to_add.size();

        Timer::Evaluate(
            [&, this]()
            {
                for (size_t i = 0; i < add_pts_size; i++)
                {
                    auto &pt = points_to_add.at(i);
                    auto key = Pos2Grid(ToEigen<double, dim>(pt));
                    auto iter = grids_map_.find(key);
                    bool update_flag = false;
                    if (iter == grids_map_.end())
                    {
                        LOG(INFO) << "not found nn in UpdateUncertainty";
                    }
                    else
                    {
                        auto pv = iter->second->second.GetPoints();
                        int pv_size = pv.size();
                        for (int j = 0; j < pv_size; j++)
                        {
                            auto pj = pv.at(j);
                            double d2 = (Eigen::Vector3d(pj.x - pt.x, pj.y - pt.y, pj.z - pt.z)).norm();
                            if (d2 < 1e-6)
                            {
                                auto pt3 = UpdateResidualOnly(pt, pj);
                                iter->second->second.ReplacePoint(j, pt3);
                                update_flag = true;
                                break;
                            }
                        }
                        if (!update_flag)
                        {
                            LOG(INFO) << "not found nn in vector";
                        }
                    }
                }
            },
            "    UpdateUncertainty");
    }

    template <int dim, IVoxNodeType node_type, typename PointType>
    void IVox<dim, node_type, PointType>::AddPoints(const PointVector &points_to_add)
    {
        int add_pts_size = points_to_add.size();
        int max_num = 1;

        std::vector<size_t> index(nearby_grids_.size());
        for (size_t j = 0; j < nearby_grids_.size(); j++)
        {
            index[j] = j;
        }
        for (int i = 0; i < add_pts_size; i++)
        {
            std::vector<DistPoint> total_nn_vec;
            total_nn_vec.clear();
            std::vector<std::vector<DistPoint>> nn_vec;
            nn_vec.clear();
            nn_vec.resize(nearby_grids_.size());
            auto &pt = points_to_add.at(i);
            auto key = Pos2Grid(ToEigen<double, dim>(pt));
            // for (const KeyType &delta: nearby_grids_)
            std::for_each(std::execution::par_unseq, index.begin(), index.end(), [&](const size_t &j)
                          {
            auto& delta = nearby_grids_.at(j);
            auto dkey = key + delta;
            auto iter = grids_map_.find(dkey);
            if (iter != grids_map_.end()) {
                iter->second->second.KNNPointMAL(nn_vec.at(j), pt, max_num, INFINITY);
            } });
            for (size_t j = 0; j < nearby_grids_.size(); j++)
            {
                if (!nn_vec.at(j).empty())
                    total_nn_vec.emplace_back(nn_vec.at(j).at(0));
            }
            if (!total_nn_vec.empty())
            {
                std::sort(total_nn_vec.begin(), total_nn_vec.end());

                auto leaf = total_nn_vec.at(0).Get();
                auto leaf_key = Pos2Grid(ToEigen<double, dim>(leaf));
                auto map_iter = grids_map_.find(leaf_key);
                if (map_iter != grids_map_.end())
                {
                    auto map_point = map_iter->second->second.GetPoint(total_nn_vec.at(0).idx);
                    auto pt_new = Merge2(pt, map_point);
                    map_iter->second->second.ErasePoint(total_nn_vec.at(0).idx);
                    grids_cache_.splice(grids_cache_.begin(), grids_cache_, map_iter->second);
                    grids_map_[leaf_key] = grids_cache_.begin();

                    auto pt_key = Pos2Grid(ToEigen<double, dim>(pt_new));
                    auto iter = grids_map_.find(pt_key);
                    if (iter == grids_map_.end())
                    {
                        grids_cache_.push_front({pt_key, NodeType(pt_new, options_.resolution_)});
                        grids_map_.insert({pt_key, grids_cache_.begin()});
                        grids_cache_.front().second.InsertPoint(pt_new);
                    }
                    else
                    {
                        iter->second->second.InsertPoint(pt_new);
                        grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
                        grids_map_[pt_key] = grids_cache_.begin();
                    }
                }
            }
            else
            {
                auto pt_key = Pos2Grid(ToEigen<double, dim>(pt));
                auto iter = grids_map_.find(pt_key);
                if (iter == grids_map_.end())
                {
                    grids_cache_.push_front({pt_key, NodeType(pt, options_.resolution_)});
                    grids_map_.insert({pt_key, grids_cache_.begin()});

                    grids_cache_.front().second.InsertPoint(pt);
                }
                else
                {
                    iter->second->second.InsertPoint(pt);
                    grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
                    grids_map_[pt_key] = grids_cache_.begin();
                }
            }
        }
    }

    template <int dim, IVoxNodeType node_type, typename PointType>
    Eigen::Matrix<int, dim, 1> IVox<dim, node_type, PointType>::Pos2Grid(const IVox::PtType &pt) const
    {
        return (pt * options_.inv_resolution_).array().floor().template cast<int>();
    }

    template <int dim, IVoxNodeType node_type, typename PointType>
    void IVox<dim, node_type, PointType>::GetMapPoints(PointVector &map_points)
    {
        map_points.clear();
        for (auto &it : grids_map_)
        {
            auto pv = it.second->second.GetPoints();
            for (const auto &pt : pv)
            {
                map_points.push_back(pt);
            }
        }
    }

} // namespace akf_lio

#endif
