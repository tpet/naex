#pragma once

#include <cmath>
#include <geometry_msgs/Transform.h>
#include <naex/clouds.h>
#include <naex/flann.h>
#include <naex/nearest_neighbors.h>
#include <naex/transforms.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>

namespace naex
{

void compute_traversability(const sensor_msgs::PointCloud2& input,
                            const geometry_msgs::Transform& transform,
                            const float radius,
                            const float max_dist,
                            const int min_support,
                            const float inclination_weight,
                            const float normal_std_weight,
                            sensor_msgs::PointCloud2& output)
{
  typedef Eigen::Transform<float, 3, Eigen::Isometry> Transform;
  assert(input.height >= 1);
  assert(input.width >= 1);
  assert(input.row_step == input.width * input.point_step);

  // Construct rotation to a fixed frame with z upward.
  Mat3 rotation = tf2::transformToEigen(transform).rotation().cast<float>();

  // Initialize output with position, normal, and traversability fields.
  output.header = input.header;
  output.height = input.height;
  output.width = input.width;
  output.is_bigendian = input.is_bigendian;
  output.is_dense = input.is_dense;
  append_position_fields<float>(output);
  append_normal_fields<float>(output);
  append_field<uint32_t>("support", 1, output);
  append_field<float>("inclination", 1, output);
  append_field<float>("normal_std", 1, output);
  append_field<float>("cost", 1, output);
  resize_cloud(output, output.height, output.width);
  if (num_points(output) == 0)
    return;

  auto position_in = flann_matrix_view<float>(const_cast<sensor_msgs::PointCloud2&>(input), "x", 3);
//  auto position_in =  const_flann_matrix_view<float>(input, "x", 3);
  auto position = flann_matrix_view<float>(output, "x", 3);
  auto normal = flann_matrix_view<float>(output, "nx", 3);
  auto support = flann_matrix_view<uint32_t>(output, "support", 1);
  auto inclination = flann_matrix_view<float>(output, "inclination", 1);
  auto normal_std = flann_matrix_view<float>(output, "normal_std", 1);
  auto cost = flann_matrix_view<float>(output, "cost", 1);
  flann::Index<flann::L2_3D<float>> index(position_in, flann::KDTreeSingleIndexParams());
  index.buildIndex();

  RadiusQuery<float> query(index, position_in, radius);
  size_t n_pts = size_t(input.height) * input.width;
  assert(query.nn_.size() == n_pts);
  assert(query.dist_.size() == n_pts);
  for (size_t i = 0; i < position_in.rows; ++i)
  {
    auto& nn = query.nn_[i];
    assert(nn.size() >= 1);
    auto& dist = query.dist_[i];
    assert(nn.size() == dist.size());
    std::copy(position_in[i], position_in[i] + 3, position[i]);
    support[i][0] = nn.size();
    if (nn.size() < 3)
    {
      std::fill(normal[i], normal[i] + 3, std::numeric_limits<float>::quiet_NaN());
      inclination[i][0] = std::numeric_limits<float>::quiet_NaN();
      normal_std[i][0] = std::numeric_limits<float>::quiet_NaN();
      cost[i][0] = std::numeric_limits<float>::quiet_NaN();
      continue;
    }
    // Estimate normals (local surface orientation).
//    Vec3 c = Vec3::Zero();
//    for (size_t j = 0; j < nn.size(); ++j)
//    {
//      ConstVec3Map p(x_in[nn[j]]);
//      c += p;
//    }
//    c /= nn.size();
    ConstVec3Map c(position_in[i]);
    Mat3 cov = Mat3::Zero();
    for (size_t j = 0; j < nn.size(); ++j)
    {
      ConstVec3Map p(position_in[nn[j]]);
      cov += (p - c) * (p - c).transpose();
    }
    cov /= std::max(1, int(nn.size() - 1));
    Eigen::SelfAdjointEigenSolver<Mat3> solver(cov);
    Vec3Map n(normal[i]);
    n = solver.eigenvectors().col(0);
    // Use fixed frame for inclination.
    inclination[i][0] = std::acos(std::abs((rotation * n)(2)));
    normal_std[i][0] = std::sqrt(solver.eigenvalues()(0));
    cost[i][0] = inclination_weight * inclination[i][0] + normal_std_weight * (normal_std[i][0] / radius);
  }
}

class Traversability
{
public:
  float radius_ = 0.5;
  float max_dist_ = 0.5;
  int min_support_ = 3;
  float inclination_weight_ = 1;
  float normal_std_weight_ = 1;
  void process(const sensor_msgs::PointCloud2& input, const geometry_msgs::Transform& transform,
               sensor_msgs::PointCloud2& output)
  {
    compute_traversability(input, transform, radius_, max_dist_, min_support_, inclination_weight_, normal_std_weight_,
                           output);
  }
};

}  // namespace naex
