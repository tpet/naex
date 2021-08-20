
#pragma once

#include <naex/cloud_filter.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>

namespace naex
{

template<typename T>
class ExcludeFramesFilter: public PointCloud2FilterFieldBase<T>
{
public:
    typedef Eigen::Matrix<T, 3, 1, Eigen::DontAlign> Vec3;
    typedef Eigen::Map<const Vec3> ConstVec3Map;

    ExcludeFramesFilter(const std::string& field,
                        const std::vector<std::string>& frames,
                        T min_range,
                        const std::shared_ptr<const tf2_ros::Buffer> buffer,
                        const ros::Duration& wait):
        PointCloud2FilterFieldBase<T>(field),
        frames_(frames),
        min_range_(min_range),
        buffer_(buffer),
        wait_(wait)
    {
        ROS_ASSERT(!frames.empty());
        ROS_ASSERT(min_range_ >= 0.0);
        ROS_ASSERT(buffer_.get() != nullptr);
        ROS_ASSERT(wait_.toSec() >= 0.0);
    }

    void prepare(const sensor_msgs::PointCloud2& input) override
    {
        positions_.clear();
        positions_.reserve(3 * frames_.size());
        for (const auto& f: frames_)
        {
            ros::Duration wait(std::max(wait_.toSec() - (ros::Time::now() - input.header.stamp).toSec(), 0.));
            const auto tf = buffer_->lookupTransform(input.header.frame_id, f, input.header.stamp, wait);
            positions_.push_back(tf.transform.translation.x);
            positions_.push_back(tf.transform.translation.y);
            positions_.push_back(tf.transform.translation.z);
        }
    }

    bool filter(const T* x)
    {
        ConstVec3Map vec(x);
        for (size_t i = 0; i + 2 < positions_.size(); i += 3)
        {
            ConstVec3Map pos(&positions_[i]);
            if ((vec - pos).norm() < min_range_)
            {
                return false;
            }
        }
        return true;
    }

protected:
    std::vector<std::string> frames_{};
    T min_range_{0.0};
    std::shared_ptr<const tf2_ros::Buffer> buffer_{};
    ros::Duration wait_{0.0};
    std::vector<T> positions_{};
};

}  // namespace naex
