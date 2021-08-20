#pragma once

//#include <naex/cloud_filter.h>
#include <naex/filter.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

namespace naex
{

template<typename T>
//class TransformFilter: public PointCloud2InPlaceFilter
class TransformProcessor: public Processor<sensor_msgs::PointCloud2>
{
public:
    typedef Eigen::Transform<T, 3, Eigen::Isometry> Transform;
    typedef Eigen::Matrix<T, 3, 1, Eigen::DontAlign> Vec3;
    typedef Eigen::Map<const Vec3> ConstVec3Map;

    TransformProcessor(const std::string& field,
                    const std::string& target,
                    const std::shared_ptr<const tf2_ros::Buffer> buffer,
                    const ros::Duration& wait):
//        PointCloud2InPlaceFilter(),
        Processor<sensor_msgs::PointCloud2>(),
//        PointCloud2FilterFieldBase<T>(field),
        field_(field),
        target_(target),
        buffer_(buffer),
        wait_(wait)
    {
        ROS_ASSERT(!field_.empty());
        ROS_ASSERT(!target_.empty());
        ROS_ASSERT(buffer_.get() != nullptr);
        ROS_ASSERT(wait_.toSec() >= 0.0);
    }
    virtual ~TransformProcessor() = default;

    void process(sensor_msgs::PointCloud2& cloud) override
    {
        ros::Duration wait(std::max(wait_.toSec() - (ros::Time::now() - cloud.header.stamp).toSec(), 0.));
        geometry_msgs::TransformStamped cloud_to_map;
        // May throw a tf2::TransformException.
        const auto to_target = buffer_->lookupTransform(target_, cloud.header.frame_id, cloud.header.stamp, wait);
        Transform transform(tf2::transformToEigen(to_target.transform));
//        transform_ = Transform(tf2::transformToEigen(to_target.transform));

        size_t n = static_cast<size_t>(cloud.height) * cloud.width;
        sensor_msgs::PointCloud2Iterator<T> it(cloud, field_);
        for (size_t i = 0; i < n; ++i, ++it)
        {
            Vec3Map x(&it[0]);
//            x = transform_ * x;
            x = transform * x;
        }
    }

protected:
    std::string field_{};
    std::string target_{};
    std::shared_ptr<const tf2_ros::Buffer> buffer_{};
    ros::Duration wait_{0.0};
//    Transform transform_;
};

}  // namespace naex
