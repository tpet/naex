
#ifndef NAEX_TRANSFORMS_H
#define NAEX_TRANSFORMS_H

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <type_traits>

namespace naex
{

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
