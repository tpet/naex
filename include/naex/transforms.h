
#ifndef NAEX_TRANSFORMS_H
#define NAEX_TRANSFORMS_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <type_traits>

namespace naex
{

template<typename F, typename I>
I quantize(F value, F scale)
{
    const auto lo = std::numeric_limits<I>::min();
    const auto hi = std::numeric_limits<I>::max();
    const F v = value / scale * hi;
    return (v < lo) ? lo : (v > hi) ? hi : static_cast<I>(v);
}

template<typename F, typename I>
F reconstruct(I value, F scale)
{
    const auto hi = std::numeric_limits<I>::max();
    return value / hi * scale;
}

template<typename F, typename I>
I quantize(F value, F min, F max)
{
    assert(min <= max);
    const auto lo = std::numeric_limits<I>::min();
    const auto hi = std::numeric_limits<I>::max();
    const F v = lo + (value - min) / (max - min) * (static_cast<F>(hi) - lo);
    return (v < lo) ? lo : (v > hi) ? hi : static_cast<I>(v);
}

template<typename F, typename I>
F reconstruct(I value, F min, F max)
{
    assert(min <= max);
    const auto lo = std::numeric_limits<I>::min();
    const auto hi = std::numeric_limits<I>::max();
    return min + (static_cast<F>(value) - lo) / (static_cast<F>(hi) - lo) * (max - min);
}

void transform_to_pose(
        const geometry_msgs::Transform& tf,
        geometry_msgs::Pose& pose)
{
    pose.position.x = tf.translation.x;
    pose.position.y = tf.translation.y;
    pose.position.z = tf.translation.z;
    pose.orientation = tf.rotation;
}

void transform_to_pose(
        const geometry_msgs::TransformStamped& tf,
        geometry_msgs::PoseStamped& pose)
{
    pose.header = tf.header;
    transform_to_pose(tf.transform, pose.pose);
}

}  // namespace naex

#endif //NAEX_TRANSFORMS_H
