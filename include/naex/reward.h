#pragma once

#include <cstddef>
#include <cmath>
#include <mutex>
#include <naex/flann.h>
#include <naex/types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
//#include <set>
#include <unordered_set>
#include <naex/voxel_filter.h>
#include <vector>

namespace naex
{

template<typename T>
T clamp(T x, T lo, T hi)
{
    return (x < lo) ? lo : (x > hi) ? hi : x;
}

template<typename P>
void suppress_reward(P& point)
{
    if (point.position_[0] >= -60. && point.position_[0] <= 0.
        && point.position_[1] >= -30. && point.position_[1] <= 30.
        && point.position_[2] >= -30. && point.position_[2] <= 30.)
    {
        Value dist_from_origin = ConstVec3Map(point.position_).norm();
        point.reward_ /= (1. + std::pow(dist_from_origin, 2.f));
    }
}

template<typename T>
T distance_coverage(T dist, T mean = 3.0, T std = 1.5)
{
    const auto z = (dist - mean) / std;
    const auto p = std::exp(-z*z);
    return p;
}

template<typename T>
T update_coverage(T prior, T prob)
{
//    prob = 1. - (1. - prior) * (1. - prob);
    prob = prob + prior - prior * prob;
    prob = clamp<T>(prob, 0.0, 1.0);
    return prob;
}

//template<typename T>
//T update_prob_visible(T prior, T dist, T mean = 3.0, T std = 1.5)
//{
//    const auto z = (dist - mean) / std;
//    // Update visibility probability.
//    auto prob = std::exp(-z*z);
////    prob = 1. - (1. - prior) * (1. - prob);
//    prob = prob + prior - prior * prob;
//    prob = clamp(prob, 0.0, 1.0);
//    return prob;
//}

class RewardPoint
{
public:
    float position_[3] = {std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN(),
                          std::numeric_limits<float>::quiet_NaN()};
    float coverage_{0.0};
    float self_coverage_{0.0};
    float reward_{0.0};
    Index support_{0};
};

/// Update point rewards at given indices using new viewpoints.
template<typename P>
void update_coverage(std::vector<P>& points,
                     const std::vector<Index>& indices,
   //                const std::vector<std::vector<size_t>>& neighborhood,
                     const std::vector<Vec3>& viewpoints,
                     Value mean = 3.0,
                     Value std = 1.5,
                     Value max_update_dist = 10,
                     bool coverage = true,
                     bool self_coverage = false)
{
    Timer t;
    for (const auto& i: indices)
    {
        auto& p = points[i];
        ConstVec3Map p_vec(p.position_);
        for (const auto& vp: viewpoints)
        {
//            ConstVec3Map vp_vec(vp.values_);
            const Value dist = (p_vec - vp).norm();
            if (dist > max_update_dist)
            {
                continue;
            }
            const auto c = distance_coverage(dist, mean, std);
            if (coverage)
            {
                p.coverage_ = update_coverage(p.coverage_, c);
            }
            if (self_coverage)
            {
                p.self_coverage_ = update_coverage(p.self_coverage_, c);
            }
        }
    }
    ROS_INFO("Coverage mask updated for %lu points from %lu viewpoints (%.3f s).",
             indices.size(), viewpoints.size(), t.seconds_elapsed());
}

/// Collect rewards at given indices using given neighborhood.
template<typename P>
//void collect_rewards(points, indices, neighborhood, float max_collect_dist = 10.0f)
void collect_rewards(std::vector<P>& points,
                     const std::vector<Index>& indices,
                     const std::vector<std::vector<Index>>& neighborhood,
                     Value mean = 3.0,
                     Value std = 1.5,
                     Value max_collect_dist = 10.0,
                     Value self_factor = 0.0)
{
    assert(neighborhood.size() == indices.size());
    Timer t;
    for (size_t i = 0; i < indices.size(); ++i)
    {
        const auto& v_i = indices[i];
        auto& p_i = points[v_i];
        p_i.reward_ = 0;
        for (const auto& v_j: neighborhood[i])
        {
            const auto& p_j = points[v_j];
            const Value dist = (ConstVec3Map(p_i.position_) - ConstVec3Map(p_j.position_)).norm();
            if (dist > max_collect_dist)
            {
                continue;
            }
            // Add the difference of previous coverage and the tested new one.
//            p_i.reward_ += update_coverage(p_j.coverage_, distance_coverage(dist, mean, std)) - p_j.coverage_;
            const Value new_coverage = update_coverage(p_j.coverage_, distance_coverage(dist, mean, std));
            const Value new_self_coverage = update_coverage(p_j.self_coverage_, distance_coverage(dist, mean, std));
            const Value reward = new_coverage - p_j.coverage_;
            const Value self_reward = new_self_coverage - p_j.self_coverage_;
            p_i.reward_ += std::max(reward, self_factor * self_reward);
        }
    }
    ROS_DEBUG("Coverage rewards collected for %lu points (%.3f s).",
              indices.size(), t.seconds_elapsed());
}

template<typename P>
void collect_rewards(std::vector<P>& points,
                     const std::vector<Index>& indices,
                     Value mean = 3.0,
                     Value std = 1.5,
                     Value max_collect_dist = 10.0,
                     Value self_factor = 0.0,
                     bool suppress_base_reward = true)
{
    // In collecting-nearby-rewards mode,
    // (a) points within D = max_vp_distance_ will have prob_visible_ updated,
    // (b) points within 2 * max_vp_distance_ will have collected reward_
    //     updated from prob_visible_ within max_vp_distance_.
    //     Edge points can yield more reward to encompass points beyond.

    // Part (a) could just use NN found via radius query below,
    // and update formula p = p0 + p1 - p0 * p1.
    // Reward is then sum of (p - p0 = p1 - p0 * p1 = p1 (1 - p0) ).

    // Part (b) is somewhat tricky as we have to imagine a viewpoint at
    // each point and compute the respective collected reward via (a).
    // Collecting rewards naively would be O(N**2), N being number of
    // points within 2 * max_vp_distance_, which would not be feasible
    // as N could be around 1e5.

    // Possible approximations:
    // (1) Subsample the points, collect reward for exemplars and distribute
    //     to the nearest neighbors (or points within the same voxel -> all
    //     points within the voxel would have same reward).
    //     radius 2*D ~ 10, diameter defined area is about 20**2 = 400 m**2,
    //     or 1600 points with 0.5 voxel size,
    //        6400 points with 0.25 voxels size,
    //        40000 points with 0.1 voxel size.
    // (2) Quantize the affected space and compute dense 3D conv via FFT.
    //     20**3 = 8000 m**3
    //     or 64000 elements with 0.5 voxels,
    //        512000 elements with 0.25 voxels,
    //        4000000 elements with 0.1 voxels.
    //     Empty voxels would have (1 - p0) = 0 so that they don't contribute.
    //     Kernel p1 would around 10**3 = 1000 m**3 or 8000 voxels.
    // (1) grows like D**2 * D**2 = D**4,
    // (2) grows like D**3 * log(D)**2.

    // (1)
    // max_vp_distance radius query
    // update coverage
    // subsample and remember pt-to-vox (trivial) mapping
    // max_vp_distance radius query with subsampled pts

    Timer t;
    // Subsample input points.
    // Compute rewards exhaustively for subsampled points.
    // Then distribute the rewards back to the represented input points.

    // Contiguous reward point buffer which can be used for kNN search.
    std::vector<RewardPoint> reward_pts;
    // Pre-allocate so that pointer survive populating the vector.
    reward_pts.reserve(indices.size());

    // Index-to-point allows distributing computed rewards to original points.
    std::vector<RewardPoint*> index_to_point;
    // Voxel-to-point allows clustering and subsampling nearby points.
    VoxelMap<int, RewardPoint*> voxel_to_point;

    Value bin_size = 0.5;
    for (const auto& i: indices)
    {
        // Construct the corresponding voxel key.
        const auto& p = points[i];
        Voxel<int> v;
        if (!v.from<Value>(p.position_, bin_size))
        {
            index_to_point.push_back(nullptr);
            continue;
        }
        // Construct or find the reward point corresponding to voxel key.
        if (voxel_to_point.find(v) == voxel_to_point.end())
        {
            RewardPoint r;
            v.center(r.position_, bin_size);
            reward_pts.push_back(r);
            voxel_to_point[v] = &reward_pts.back();
        }
        // Update the reward point with current point.
        RewardPoint* r_ptr = voxel_to_point[v];
        index_to_point.push_back(r_ptr);
        // Accumulate coverage each voxel to compute mean.
        r_ptr->coverage_ += p.coverage_;
        r_ptr->support_ += 1;
    }

    // Compute mean coverage.
    for (auto& p: reward_pts)
    {
        if (p.support_ > 0)
        {
            p.coverage_ /= p.support_;
        }
    }

    // Create neighborhood graph for subsampled points.
    flann::Matrix<float> mat(reward_pts[0].position_, reward_pts.size(), 3, sizeof(RewardPoint));
    auto index = flann::Index<flann::L2_3D<Value>>(mat, flann::KDTreeSingleIndexParams());
    index.buildIndex();
    RadiusQuery<Value> q(index, mat, max_collect_dist);
    std::vector<Index> all;
    index_range(q.nn_.size(), all);

    collect_rewards(reward_pts, all, q.nn_, mean, std, max_collect_dist);

    // Distribute rewards to original points.
    for (Index i = 0; i < indices.size(); ++i)
    {
        const RewardPoint* r_ptr = index_to_point[i];
        if (r_ptr == nullptr)
        {
            continue;
        }
        // Assume constant rewards from all represented points.
        auto& pt = points[indices[i]];
        pt.reward_ = r_ptr->support_ * r_ptr->reward_;
        if (suppress_base_reward)
        {
            suppress_reward(pt);
        }
    }

    ROS_INFO("Collected rewards for %lu input points or %lu %.2f-m voxels (%.3f s).",
             indices.size(), reward_pts.size(), bin_size, t.seconds_elapsed());
}

}  // namespace naex
