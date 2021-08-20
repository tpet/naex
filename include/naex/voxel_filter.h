
#pragma once

#include <cmath>
//#include <naex/cloud_filter.h>
#include <naex/filter.h>
//#include <naex/geom.h>
#include <naex/timer.h>
#include <naex/types.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <unordered_set>
#include <unordered_map>

namespace naex
{

template<typename C>
void index_range(size_t n, C& indices)
{
//    Timer t;
    indices.reserve(n);
    for (size_t i = 0; i < n; ++i)
    {
        indices.push_back(i);
    }
    indices.resize(n);
//    ROS_DEBUG("Index range of size %lu created (%.6f s).", n, t.seconds_elapsed());
}

template<typename It>
void shuffle(It begin, It end)
{
//    Timer t;
    std::random_device rd;
    std::mt19937 g(rd());
    std::shuffle(begin, end, g);
//    ROS_DEBUG("%lu points shuffled (%.6f s).", n, t.seconds_elapsed());
}

template<typename C>
void random_permutation(size_t n, C& indices)
{
    Timer t;
    index_range(n, indices);
    shuffle(indices.begin(), indices.end());
    ROS_DEBUG("Random permutation of %lu points created (%.6f s).",
              n, t.seconds_elapsed());
}

template <class T>
inline void hash_combine(std::size_t& seed, const T& v)
{
    // Boost-like hash combine
    std::hash<T> hasher;
    seed ^= hasher(v) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
}

template<typename I>
class Voxel
{
public:
    typedef Voxel<I> Same;

    class Hash
    {
    public:
        size_t operator()(const Same& voxel) const noexcept
        {
            return voxel.hash();
        }
    };

    Voxel()
    {}
    Voxel(I x, I y, I z):
        x_(x), y_(y), z_(z)
    {}
    Voxel(const Same& other):
        x_(other.x_), y_(other.y_), z_(other.z_)
    {}
    size_t hash() const noexcept
    {
        size_t h = 0;
        hash_combine(h, x_);
        hash_combine(h, y_);
        hash_combine(h, z_);
        return h;
    }
    friend bool operator==(const Same& a, const Same& b)
    {
        return (a.x_ == b.x_) && (a.y_ == b.y_) && (a.z_ == b.z_);
    }

    static I lb() { return std::numeric_limits<I>::min(); }
    static I ub() { return std::numeric_limits<I>::max(); }

    template<typename T>
    bool from(const T* values, T bin_size)
    {
        if (!std::isfinite(values[0]) || !std::isfinite(values[1]) || !std::isfinite(values[2]))
        {
            return false;
        }
        // TODO: Saturate cast.
        Value x = std::floor(values[0] / bin_size);
        Value y = std::floor(values[1] / bin_size);
        Value z = std::floor(values[2] / bin_size);
        if (x < lb() || x > ub() || y < lb() || y > ub() || z < lb() || z > ub())
        {
            return false;
        }
        x_ = static_cast<I>(x);
        y_ = static_cast<I>(y);
        z_ = static_cast<I>(z);
        return true;
    }
    template<typename T>
    void center(T* values, T bin_size)
    {
        values[0] = (T(x_) + 0.5) * bin_size;
        values[1] = (T(y_) + 0.5) * bin_size;
        values[2] = (T(z_) + 0.5) * bin_size;
    }

    I x_{0};
    I y_{0};
    I z_{0};
};

template<typename I>
using VoxelVec = std::vector<Voxel<I>>;

template<typename I>
using VoxelSet = std::unordered_set<Voxel<I>, typename Voxel<I>::Hash>;

template<typename I, typename T>
using VoxelMap = std::unordered_map<Voxel<I>, T, typename Voxel<I>::Hash>;

template<typename T, typename I>
void voxel_filter(const sensor_msgs::PointCloud2& input,
                  const std::string& field,
                  const T bin_size,
                  sensor_msgs::PointCloud2& output,
                  VoxelSet<I>& voxels)
{
    Timer t, t_part;
    assert(input.row_step % input.point_step == 0);
    const Index n_pts = input.height * input.width;

    std::vector<Index> indices;
    random_permutation(n_pts, indices);

    t_part.reset();
    std::vector<Index> keep;
    keep.reserve(indices.size());
    sensor_msgs::PointCloud2ConstIterator<T> pt_it(input, field);
    voxels.reserve(keep.size());
    for (Index i = 0; i < n_pts; ++i, ++pt_it)
    {
        Voxel<I> voxel;
        if (!voxel.from(&pt_it[0], bin_size))
            continue;

        if (voxels.find(voxel) == voxels.end())
        {
            voxels.insert(voxel);
            keep.push_back(i);
        }
    }
    ROS_DEBUG("Getting %lu indices to keep (%.6f s).", keep.size(), t_part.seconds_elapsed());

    // Copy selected indices.
    t_part.reset();
    copy_points(input, keep, output);
    ROS_DEBUG("%lu / %lu points kept by voxel filter (%.6f s).",
              keep.size(), size_t(n_pts), t_part.seconds_elapsed());
}

template<typename T, typename I>
//class VoxelFilter: public PointCloud2Filter
class VoxelFilter: public Filter<sensor_msgs::PointCloud2>
{
public:
    VoxelFilter(const std::string& field, T bin_size):
//        PointCloud2Filter(),
        Filter<sensor_msgs::PointCloud2>(),
        field_(field),
        bin_size_(bin_size)
    {
        ROS_ASSERT(std::isfinite(bin_size) && bin_size > 0.0);
    }
    virtual ~VoxelFilter() = default;

    void filter(const sensor_msgs::PointCloud2& input, sensor_msgs::PointCloud2& output) override
    {
        VoxelSet<I> voxels;
        voxel_filter<T, I>(input, field_, bin_size_, output, voxels);
    }

protected:
    std::string field_;
    T bin_size_;
};

}  // namespace naex
