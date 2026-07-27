#include <pcl/common/centroid.h>
#include <algorithm>
#include <cmath>
#include <list>
#include <vector>

#include "hilbert.hpp"

namespace akf_lio
{

    template <typename PointT>
    inline PointT UpdateResidualOnly(const PointT &pt1, const PointT &pt2)
    {
        PointT pt3 = pt2;

        int use_sum = pt1.use_num + pt2.use_num;
        if (use_sum != 0)
        {
            double ratio_3 = (double)pt1.use_num / (double)(use_sum);
            double ratio_4 = (double)pt2.use_num / (double)(use_sum);
            pt3.uncertainty = ratio_3 * pt1.uncertainty + ratio_4 * pt2.uncertainty;
            pt3.use_num = use_sum;
            if (pt3.use_num > options::MAX_FEA_NUM)
                pt3.use_num = options::MAX_FEA_NUM;
        }
        return pt3;
    }

    template <typename PointT>
    inline PointT Merge2(const PointT &pt1, const PointT &pt2)
    {
        PointT pt3 = pt2;
        common::V3D p1(pt1.x, pt1.y, pt1.z), p2(pt2.x, pt2.y, pt2.z), p3;
        double ratio_1 = (double)pt1.pt_num / (double)(pt1.pt_num + pt2.pt_num);
        double ratio_2 = (double)pt2.pt_num / (double)(pt1.pt_num + pt2.pt_num);

        p3 = p1 * ratio_1 + p2 * ratio_2;
        pt3.x = p3(0);
        pt3.y = p3(1);
        pt3.z = p3(2);
        pt3.intensity = (pt1.intensity * ratio_1 + pt2.intensity * ratio_2);
        pt3.cov = ratio_1 * (pt1.cov + p1 * p1.transpose()) +
                  ratio_2 * (pt2.cov + p2 * p2.transpose()) - p3 * p3.transpose();
        pt3.pt_num = pt1.pt_num + pt2.pt_num;
        pt3.time = pt1.time > pt2.time ? pt1.time : pt2.time;
        if (pt3.pt_num > options::MAX_FEA_NUM)
            pt3.pt_num = options::MAX_FEA_NUM;

        int use_sum = pt1.use_num + pt2.use_num;
        if (use_sum != 0)
        {
            double ratio_3 = (double)pt1.use_num / (double)(use_sum);
            double ratio_4 = (double)pt2.use_num / (double)(use_sum);
            common::V3D p4 = ratio_3 * p1 + ratio_4 * p2;
            pt3.uncertainty = (ratio_3 * pt1.uncertainty + ratio_4 * pt2.uncertainty);
            pt3.use_num = use_sum;
            if (pt3.use_num > options::MAX_FEA_NUM)
                pt3.use_num = options::MAX_FEA_NUM;
        }
        return pt3;
    }

    template <typename PointT>
    inline double Distance2(const PointT &pt1, const PointT &pt2)
    {
        Eigen::Vector3d d = Eigen::Vector3d(pt1.x, pt1.y, pt1.z) - Eigen::Vector3d(pt2.x, pt2.y, pt2.z);
        return d.norm();
    }

    template <typename PointT>
    inline double MalDistance(const PointT &pt1, const PointT &pt2)
    {
        Eigen::Vector3d d = Eigen::Vector3d(pt1.x, pt1.y, pt1.z) - Eigen::Vector3d(pt2.x, pt2.y, pt2.z);
        double mal_dis = d.transpose() * (pt1.cov + pt2.cov).inverse() * d;
        return mal_dis;
    }

    // convert from pcl point to eigen
    template <typename T, int dim, typename PointType>
    inline Eigen::Matrix<T, dim, 1> ToEigen(const PointType &pt)
    {
        return Eigen::Matrix<T, dim, 1>(pt.x, pt.y, pt.z);
    }

    template <typename PointT, int dim = 3>
    class IVoxNode
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        struct DistPoint;

        IVoxNode() = default;
        IVoxNode(const PointT &center, const double &side_length) {} /// same with phc

        void InsertPoint(const PointT &pt);
        inline void Clear();
        inline bool Empty() const;

        inline std::size_t Size() const;

        inline PointT GetPoint(const std::size_t idx) const;

        inline std::vector<PointT> GetPoints() const;

        inline void ErasePoint(const size_t idx);

        inline void ReplacePoint(const std::size_t idx, const PointT &pt);

        int KNNPointByCondition(std::vector<DistPoint> &dis_points, const PointT &point, const int &K,
                                const double &max_range);
        int KNNPointMAL(std::vector<DistPoint> &dis_points, const PointT &point, const int &K,
                        const double &max_range);
        //   private:
        std::vector<PointT> points_;
    };

    template <typename PointT, int dim = 3>
    class IVoxNodePhc
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        struct DistPoint;
        struct PhcCube;

        IVoxNodePhc() = default;
        IVoxNodePhc(const PointT &center, const double &side_length, const int &phc_order = 6);

        void InsertPoint(const PointT &pt);

        void ErasePoint(const PointT &pt, const double erase_distance_th_);

        inline bool Empty() const;

        inline std::size_t Size() const;

        PointT GetPoint(const std::size_t idx) const;

        bool NNPoint(const PointT &cur_pt, DistPoint &dist_point) const;

        int KNNPointByCondition(std::vector<DistPoint> &dis_points, const PointT &cur_pt, const int &K = 5,
                                const double &max_range = 5.0);

    private:
        uint32_t CalculatePhcIndex(const PointT &pt) const;

    private:
        std::vector<PhcCube> phc_cubes_;

        PointT center_;
        double side_length_ = 0;
        int phc_order_ = 6;
        double phc_side_length_ = 0;
        double phc_side_length_inv_ = 0;
        Eigen::Matrix<double, dim, 1> min_cube_;
    };

    template <typename PointT, int dim>
    struct IVoxNode<PointT, dim>::DistPoint
    {
        double dist = 0;
        IVoxNode *node = nullptr;
        int idx = 0;

        DistPoint() = default;
        DistPoint(const double d, IVoxNode *n, const int i) : dist(d), node(n), idx(i) {}

        PointT Get() { return node->GetPoint(idx); }

        inline bool operator()(const DistPoint &p1, const DistPoint &p2) { return p1.dist < p2.dist; }

        inline bool operator<(const DistPoint &rhs) { return dist < rhs.dist; }
    };

    template <typename PointT, int dim>
    void IVoxNode<PointT, dim>::InsertPoint(const PointT &pt)
    {
        points_.template emplace_back(pt);
    }

    template <typename PointT, int dim>
    void IVoxNode<PointT, dim>::Clear()
    {
        points_.clear();
    }

    template <typename PointT, int dim>
    bool IVoxNode<PointT, dim>::Empty() const
    {
        return points_.empty();
    }

    template <typename PointT, int dim>
    std::size_t IVoxNode<PointT, dim>::Size() const
    {
        return points_.size();
    }

    template <typename PointT, int dim>
    PointT IVoxNode<PointT, dim>::GetPoint(const std::size_t idx) const
    {
        return points_[idx];
    }

    template <typename PointT, int dim>
    std::vector<PointT> IVoxNode<PointT, dim>::GetPoints() const
    {
        return points_;
    }

    template <typename PointT, int dim>
    void IVoxNode<PointT, dim>::ReplacePoint(const std::size_t idx, const PointT &pt)
    {
        points_.at(idx) = pt;
    }

    template <typename PointT, int dim>
    void IVoxNode<PointT, dim>::ErasePoint(const std::size_t idx)
    {
        //    LOG(INFO)<<" before erase "<<points_.size();
        //    for (auto &p:points_) {
        //        LOG(INFO)<<" before erase "<<p.x;
        //    }
        points_.erase(points_.begin() + idx);
        //    LOG(INFO)<<" after erase "<<points_.size();
        //    for (auto &p:points_) {
        //        LOG(INFO)<<" after erase "<<p.x;
        //    }
        //    points_.shrink_to_fit();
    }

    template <typename PointT, int dim>
    int IVoxNode<PointT, dim>::KNNPointByCondition(std::vector<DistPoint> &dis_points, const PointT &point, const int &K,
                                                   const double &max_range)
    {
        std::size_t old_size = dis_points.size();

        for (const auto &pt : points_)
        {
            double d = MalDistance(pt, point);

            {
                dis_points.template emplace_back(DistPoint(d, this, &pt - points_.data()));
            }
        }

        // sort by distance
        if (old_size + K >= dis_points.size())
        {
        }
        else
        {
            std::nth_element(dis_points.begin() + old_size, dis_points.begin() + old_size + K - 1, dis_points.end());
            dis_points.resize(old_size + K);
        }

        return dis_points.size();
    }

    template <typename PointT, int dim>
    int IVoxNode<PointT, dim>::KNNPointMAL(std::vector<DistPoint> &dis_points, const PointT &point, const int &K,
                                           const double &max_range)
    {
        std::size_t old_size = dis_points.size();

        for (const auto &pt : points_)
        {
            double d = MalDistance(pt, point);
            if (d < options::T_MAL)
            {
                dis_points.template emplace_back(DistPoint(d, this, &pt - points_.data()));
            }
        }
        // sort by distance
        if (old_size + K >= dis_points.size())
        {
        }
        else
        {
            std::nth_element(dis_points.begin() + old_size, dis_points.begin() + old_size + K - 1, dis_points.end());
            dis_points.resize(old_size + K);
        }
        return dis_points.size();
    }

    template <typename PointT, int dim>
    struct IVoxNodePhc<PointT, dim>::DistPoint
    {
        double dist = 0;
        IVoxNodePhc *node = nullptr;
        int idx = 0;

        DistPoint() {}
        DistPoint(const double d, IVoxNodePhc *n, const int i) : dist(d), node(n), idx(i) {}

        PointT Get() { return node->GetPoint(idx); }

        inline bool operator()(const DistPoint &p1, const DistPoint &p2) { return p1.dist < p2.dist; }

        inline bool operator<(const DistPoint &rhs) { return dist < rhs.dist; }
    };

    template <typename PointT, int dim>
    struct IVoxNodePhc<PointT, dim>::PhcCube
    {
        uint32_t idx = 0;
        pcl::CentroidPoint<PointT> mean;

        PhcCube(uint32_t index, const PointT &pt) { mean.add(pt); }

        void AddPoint(const PointT &pt) { mean.add(pt); }

        PointT GetPoint() const
        {
            PointT pt;
            mean.get(pt);
            return std::move(pt);
        }
    };

    template <typename PointT, int dim>
    IVoxNodePhc<PointT, dim>::IVoxNodePhc(const PointT &center, const double &side_length, const int &phc_order)
        : center_(center), side_length_(side_length), phc_order_(phc_order)
    {
        assert(phc_order <= 8);
        phc_side_length_ = side_length_ / (std::pow(2, phc_order_));
        phc_side_length_inv_ = (std::pow(2, phc_order_)) / side_length_;
        min_cube_ = center_.getArray3fMap() - side_length / 2.0;
        phc_cubes_.reserve(64);
    }

    template <typename PointT, int dim>
    void IVoxNodePhc<PointT, dim>::InsertPoint(const PointT &pt)
    {
        uint32_t idx = CalculatePhcIndex(pt);

        PhcCube cube{idx, pt};
        auto it = std::lower_bound(phc_cubes_.begin(), phc_cubes_.end(), cube,
                                   [](const PhcCube &a, const PhcCube &b)
                                   { return a.idx < b.idx; });

        if (it == phc_cubes_.end())
        {
            phc_cubes_.emplace_back(cube);
        }
        else
        {
            if (it->idx == idx)
            {
                it->AddPoint(pt);
            }
            else
            {
                phc_cubes_.insert(it, cube);
            }
        }
    }

    template <typename PointT, int dim>
    void IVoxNodePhc<PointT, dim>::ErasePoint(const PointT &pt, const double erase_distance_th_)
    {
        uint32_t idx = CalculatePhcIndex(pt);

        PhcCube cube{idx, pt};
        auto it = std::lower_bound(phc_cubes_.begin(), phc_cubes_.end(), cube,
                                   [](const PhcCube &a, const PhcCube &b)
                                   { return a.idx < b.idx; });

        if (erase_distance_th_ > 0)
        {
        }
        if (it != phc_cubes_.end() && it->idx == idx)
        {
            phc_cubes_.erase(it);
        }
    }

    template <typename PointT, int dim>
    bool IVoxNodePhc<PointT, dim>::Empty() const
    {
        return phc_cubes_.empty();
    }

    template <typename PointT, int dim>
    std::size_t IVoxNodePhc<PointT, dim>::Size() const
    {
        return phc_cubes_.size();
    }

    template <typename PointT, int dim>
    PointT IVoxNodePhc<PointT, dim>::GetPoint(const std::size_t idx) const
    {
        return phc_cubes_[idx].GetPoint();
    }

    template <typename PointT, int dim>
    bool IVoxNodePhc<PointT, dim>::NNPoint(const PointT &cur_pt, DistPoint &dist_point) const
    {
        if (phc_cubes_.empty())
        {
            return false;
        }
        uint32_t cur_idx = CalculatePhcIndex(cur_pt);
        PhcCube cube{cur_idx, cur_pt};
        auto it = std::lower_bound(phc_cubes_.begin(), phc_cubes_.end(), cube,
                                   [](const PhcCube &a, const PhcCube &b)
                                   { return a.idx < b.idx; });

        if (it == phc_cubes_.end())
        {
            it--;
            dist_point = DistPoint(Distance2(cur_pt, it->GetPoint()), this, it - phc_cubes_.begin());
        }
        else if (it == phc_cubes_.begin())
        {
            dist_point = DistPoint(Distance2(cur_pt, it->GetPoint()), this, it - phc_cubes_.begin());
        }
        else
        {
            auto last_it = it;
            last_it--;
            double d1 = Distance2(cur_pt, it->GetPoint());
            double d2 = Distance2(cur_pt, last_it->GetPoint());
            if (d1 > d2)
            {
                dist_point = DistPoint(d2, this, it - phc_cubes_.begin());
            }
            else
            {
                dist_point = DistPoint(d1, this, it - phc_cubes_.begin());
            }
        }

        return true;
    }

    template <typename PointT, int dim>
    int IVoxNodePhc<PointT, dim>::KNNPointByCondition(std::vector<DistPoint> &dis_points, const PointT &cur_pt,
                                                      const int &K, const double &max_range)
    {
        uint32_t cur_idx = CalculatePhcIndex(cur_pt);
        PhcCube cube{cur_idx, cur_pt};
        auto it = std::lower_bound(phc_cubes_.begin(), phc_cubes_.end(), cube,
                                   [](const PhcCube &a, const PhcCube &b)
                                   { return a.idx < b.idx; });

        const int max_search_cube_side_length = std::pow(2, std::ceil(std::log2(max_range * phc_side_length_inv_)));
        const int max_search_idx_th =
            8 * max_search_cube_side_length * max_search_cube_side_length * max_search_cube_side_length;

        auto create_dist_point = [&cur_pt, this](typename std::vector<PhcCube>::const_iterator forward_it)
        {
            double d = Distance2(forward_it->GetPoint(), cur_pt);
            return DistPoint(d, this, forward_it - phc_cubes_.begin());
        };

        typename std::vector<PhcCube>::const_iterator forward_it(it);
        typename std::vector<PhcCube>::const_reverse_iterator backward_it(it);
        if (it != phc_cubes_.end())
        {
            dis_points.emplace_back(create_dist_point(it));
            forward_it++;
        }
        if (backward_it != phc_cubes_.rend())
        {
            backward_it++;
        }

        auto forward_reach_boundary = [&]()
        {
            return forward_it == phc_cubes_.end() || forward_it->idx - cur_idx > max_search_idx_th;
        };
        auto backward_reach_boundary = [&]()
        {
            return backward_it == phc_cubes_.rend() || cur_idx - backward_it->idx > max_search_idx_th;
        };

        while (!forward_reach_boundary() && !backward_reach_boundary())
        {
            if (forward_it->idx - cur_idx > cur_idx - backward_it->idx)
            {
                dis_points.emplace_back(create_dist_point(forward_it));
                forward_it++;
            }
            else
            {
                dis_points.emplace_back(create_dist_point(backward_it.base()));
                backward_it++;
            }
            if (dis_points.size() > K)
            {
                break;
            }
        }

        if (forward_reach_boundary())
        {
            while (!backward_reach_boundary() && dis_points.size() < K)
            {
                dis_points.emplace_back(create_dist_point(backward_it.base()));
                backward_it++;
            }
        }

        if (backward_reach_boundary())
        {
            while (!forward_reach_boundary() && dis_points.size() < K)
            {
                dis_points.emplace_back(create_dist_point(forward_it));
                forward_it++;
            }
        }

        return dis_points.size();
    }

    template <typename PointT, int dim>
    uint32_t IVoxNodePhc<PointT, dim>::CalculatePhcIndex(const PointT &pt) const
    {
        Eigen::Matrix<double, dim, 1> eposf = (pt.getVector3fMap() - min_cube_) * phc_side_length_inv_;
        Eigen::Matrix<int, dim, 1> eposi = eposf.template cast<int>();
        for (int i = 0; i < dim; ++i)
        {
            if (eposi(i, 0) < 0)
            {
                eposi(i, 0) = 0;
            }
            if (eposi(i, 0) > std::pow(2, phc_order_))
            {
                eposi(i, 0) = std::pow(2, phc_order_) - 1;
            }
        }
        std::array<uint8_t, 3> apos{eposi(0), eposi(1), eposi(2)};
        std::array<uint8_t, 3> tmp = hilbert::v2::PositionToIndex(apos);

        uint32_t idx = (uint32_t(tmp[0]) << 16) + (uint32_t(tmp[1]) << 8) + (uint32_t(tmp[2]));
        return idx;
    }

} // namespace akf_lio
