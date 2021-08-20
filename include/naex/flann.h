#pragma once

#include <flann/flann.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>

namespace naex
{

template<typename T>
flann::Matrix<T> flann_matrix_view(sensor_msgs::PointCloud2& cloud,
                                   const std::string& field,
                                   const size_t count = 1)
{
    sensor_msgs::PointCloud2Iterator<T> it(cloud, field);
    return flann::Matrix<T>(&it[0], static_cast<size_t>(cloud.height) * cloud.width, count, cloud.point_step);
}

template<typename T>
flann::Matrix<const T> const_flann_matrix_view(const sensor_msgs::PointCloud2& cloud,
                                               const std::string& field,
                                               const size_t count = 1)
{
    sensor_msgs::PointCloud2ConstIterator<T> it(cloud, field);
    return flann::Matrix<const T>(&it[0], static_cast<size_t>(cloud.height) * cloud.width, count, cloud.point_step);
}

}  // namespace naex
