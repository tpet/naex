#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <naex/clouds.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <type_traits>

namespace naex
{

template<class T>
void transform_cloud(const sensor_msgs::PointCloud2& input,
                     const geometry_msgs::Transform& transform,
                     sensor_msgs::PointCloud2& output)
{
  typedef Eigen::Transform<T, 3, Eigen::Isometry> Transform;
  typedef Eigen::Matrix<T, 3, 1, Eigen::DontAlign> Vec3;
  typedef Eigen::Map<Vec3> Vec3Map;

  Transform tf(tf2::transformToEigen(transform));

  output = input;
  const auto n = num_points(output);
  sensor_msgs::PointCloud2Iterator<T> x_out(output, "x");
  for (size_t i = 0; i < n; ++i, ++x_out)
  {
    Vec3Map x(&x_out[0]);
    x = tf * x;
  }
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
