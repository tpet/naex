
#pragma once

#include <naex/cloud_filter.h>
#include <ros/ros.h>

namespace naex
{

template<typename T>
class RangeFilter: public PointCloud2FilterFieldBase<T>
{
public:
    typedef Eigen::Matrix<T, 3, 1, Eigen::DontAlign> Vec3;
    typedef Eigen::Map<const Vec3> ConstVec3Map;

    explicit RangeFilter(const std::string& field,
                         T min_range = 0.0,
                         T max_range = std::numeric_limits<T>::infinity()):
        PointCloud2FilterFieldBase<T>(field),
        min_range_(min_range),
        max_range_(max_range)
    {}

    bool filter(const T* x)
    {
        ConstVec3Map vec(x);
        const T range = vec.norm();
        return range >= min_range_ && range <= max_range_;
    }

protected:
    std::vector<std::string> frames_{};
    T min_range_{0.0};
    T max_range_{std::numeric_limits<T>::infinity()};
};

}  // namespace naex
