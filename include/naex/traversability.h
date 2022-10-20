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

/**
 * @brief Traversability estimation.
 *
 * @param input Input point cloud.
 * @param transform Transform to fixed frame.
 * @param support_radius Local support radius.
 * @param min_support Minimum support to consider point further.
 * @param inclination_radius Radius for local surface orientation.
 * @param inclination_weight Weight of inclination in cost.
 * @param normal_std_weight Weight of normal standard deviation in cost.
 * @param clearance_radius Radius of clearance cylinder.
 * @param clearance_low Height of cylinder base.
 * @param clearance_high Height of cylinder top.
 * @param obstacle_weight Weight of each obstacle point.
 * @param output Output point cloud.
 */
void compute_traversability(const sensor_msgs::PointCloud2 & input,
                            const geometry_msgs::Transform & transform,
                            const float min_z,
                            const float max_z,
                            const float support_radius,
                            const int min_support,
                            const float inclination_radius,
                            const float inclination_weight,
                            const float normal_std_weight,
                            const float clearance_radius_,
                            const float clearance_low_,
                            const float clearance_high_,
                            const float obstacle_weight,
                            sensor_msgs::PointCloud2 & output)
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
    append_field<uint32_t>("obstacles", 1, output);
    append_field<float>("cost", 1, output);
    resize_cloud(output, output.height, output.width);
    if (num_points(output) == 0)
        return;

    auto position_in = flann_matrix_view<float>(const_cast<sensor_msgs::PointCloud2 &>(input), "x", 3);
//  auto position_in =  const_flann_matrix_view<float>(input, "x", 3);
    auto position = flann_matrix_view<float>(output, "x", 3);
    auto normal = flann_matrix_view<float>(output, "nx", 3);
    auto support = flann_matrix_view<uint32_t>(output, "support", 1);
    auto inclination = flann_matrix_view<float>(output, "inclination", 1);
    auto normal_std = flann_matrix_view<float>(output, "normal_std", 1);
    auto obstacles = flann_matrix_view<uint32_t>(output, "obstacles", 1);
    auto cost = flann_matrix_view<float>(output, "cost", 1);
    flann::Index<flann::L2_3D<float>> index(position_in, flann::KDTreeSingleIndexParams());
    index.buildIndex();

    float support_radius2 = support_radius * support_radius;
    float inclination_radius2 = inclination_radius * inclination_radius;
    float radius = std::max(std::max(inclination_radius,
                                     support_radius),
                            std::hypot(clearance_radius_, clearance_high_));

    RadiusQuery<float> query(index, position_in, radius, -1);
    size_t n_pts = num_points(input);
    assert(query.nn_.size() == n_pts);
    assert(query.dist_.size() == n_pts);

    // First pass: copy position and compute local support.
    for (size_t i = 0; i < position_in.rows; ++i)
    {
        auto & nn = query.nn_[i];
        assert(nn.size() >= 1);
        auto & dist = query.dist_[i];
        assert(nn.size() == dist.size());
        std::copy(position_in[i], position_in[i] + 3, position[i]);
        support[i][0] = 0;
        ConstVec3Map p(position[i]);
        if (std::isfinite(min_z) && (rotation * p)(2) < min_z)
            continue;
        if (std::isfinite(max_z) && (rotation * p)(2) > max_z)
            continue;
        for (size_t j = 0; j < dist.size() && dist[j] <= support_radius2; ++j)
        {
            ++support[i][0];
        }
    }

    // Second pass: estimate normal, inclination, and clearance.
    for (size_t i = 0; i < position_in.rows; ++i)
    {
        if (support[i][0] < min_support)
        {
            std::fill(normal[i], normal[i] + 3, std::numeric_limits<float>::quiet_NaN());
            inclination[i][0] = std::numeric_limits<float>::quiet_NaN();
            normal_std[i][0] = std::numeric_limits<float>::quiet_NaN();
            obstacles[i][0] = 0;
            cost[i][0] = std::numeric_limits<float>::quiet_NaN();
            continue;
        }

        auto & nn = query.nn_[i];
        auto & dist = query.dist_[i];
        // Center with mean.
//        Vec3 c = Vec3::Zero();
//        for (size_t j = 0; j < nn.size(); ++j)
//        {
//          ConstVec3Map p(x_in[nn[j]]);
//          c += p;
//        }
//        c /= nn.size();
        // Center with current point.
        ConstVec3Map c(position[i]);
        Mat3 cov = Mat3::Zero();
        int n_cov = 0;
        for (size_t j = 0; j < nn.size() && dist[j] <= inclination_radius2; ++j)
        {
            if (support[nn[j]][0] < min_support)
                continue;
            ConstVec3Map p(position[nn[j]]);
            cov += (p - c) * (p - c).transpose();
            ++n_cov;
        }
        cov /= (n_cov > 1) ? (n_cov - 1) : 1;
        Eigen::SelfAdjointEigenSolver<Mat3> solver(cov);
        Vec3Map n(normal[i]);
        n = solver.eigenvectors().col(0);
        // Use fixed frame for inclination.
        inclination[i][0] = std::acos(std::abs((rotation * n)(2)));
        normal_std[i][0] = std::sqrt(solver.eigenvalues()(0));
        // Find obstacles in clearance cylinder around upward pointing normal.
        if ((rotation * n)(2) < 0)
            n = -n;
        obstacles[i][0] = 0;
        for (size_t j = 0; j < nn.size(); ++j)
        {
            if (support[nn[j]][0] < min_support)
                continue;
            ConstVec3Map p(position[nn[j]]);
            Value height_diff = n.dot(p - c);
            Vec3 ground_pt = p - height_diff * n;
            Value ground_dist = (ground_pt - c).norm();
            if (ground_dist <= clearance_radius_
                    && height_diff >= clearance_low_
                    && height_diff <= clearance_high_)
            {
                ++obstacles[i][0];
            }
        }
        cost[i][0] = inclination_weight * inclination[i][0]
                + normal_std_weight * normal_std[i][0]
                + obstacle_weight * obstacles[i][0];
    }
}

void remove_low_support(const sensor_msgs::PointCloud2 & input,
                        const int min_support,
                        sensor_msgs::PointCloud2 & output)
{
    std::vector<Index> keep;
    keep.reserve(num_points(input));
    sensor_msgs::PointCloud2ConstIterator<uint32_t> it(input, "support");
    for (size_t i = 0; i < num_points(input); ++i, ++it)
    {
        if (it[0] >= min_support)
            keep.push_back(i);
    }
    copy_points(input, keep, output);
}

class Traversability
{
public:
//    float min_x_ = std::numeric_limits<float>::quiet_NaN();
//    float max_x_ = std::numeric_limits<float>::quiet_NaN();
//    float min_y_ = std::numeric_limits<float>::quiet_NaN();
//    float max_y_ = std::numeric_limits<float>::quiet_NaN();
    float min_z_ = std::numeric_limits<float>::quiet_NaN();
    float max_z_ = std::numeric_limits<float>::quiet_NaN();
//    float min_range_ = std::numeric_limits<float>::quiet_NaN();
//    float max_range_ = std::numeric_limits<float>::quiet_NaN();
//    float voxel_size_ = std::numeric_limits<float>::quiet_NaN();

    float support_radius_ = 0.25;
    int min_support_ = 3;

    float inclination_radius_ = 0.5;
    float inclination_weight_ = 1;
    float normal_std_weight_ = 1;

    float clearance_radius_ = 0.5;
    float clearance_low_ = 0.1;
    float clearance_high_ = 0.5;
    float obstacle_weight_ = 1;

    bool remove_low_support_ = false;

    void process(const sensor_msgs::PointCloud2 & input, const geometry_msgs::Transform & transform,
                 sensor_msgs::PointCloud2 & output)
    {
        // TODO: Apply box, range, and voxel filters.
        sensor_msgs::PointCloud2 traversability;
        compute_traversability(input, transform,
                               min_z_, max_z_, support_radius_, min_support_,
                               inclination_radius_, inclination_weight_, normal_std_weight_,
                               clearance_radius_, clearance_low_, clearance_high_, obstacle_weight_,
                               traversability);
        if (remove_low_support_)
            remove_low_support(traversability, min_support_, output);
        else
            output = traversability;
    }
};

}  // namespace naex
