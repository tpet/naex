//
// Created by petrito1 on 10/1/20.
//

#ifndef NAEX_PLANNER_H
#define NAEX_PLANNER_H

#include <algorithm>
#include <boost/graph/graph_concepts.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <mutex>
#include <naex/buffer.h>
#include <naex/timer.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace naex
{
    // Point cloud position and normal element type
    typedef float Elem;
    typedef Eigen::Matrix<Elem, 3, 1, Eigen::DontAlign> Vec3;
    typedef Eigen::Map<Vec3> Vec3Map;
    typedef Eigen::Map<const Vec3> ConstVec3Map;
    typedef Eigen::Matrix<Elem, 2, 1, Eigen::DontAlign> Vec2;
    typedef Eigen::Map<Vec2> Vec2Map;
    typedef Eigen::Map<const Vec3> ConstVec2Map;
    typedef Eigen::Matrix<Elem, 3, 3, Eigen::DontAlign> Mat3;
    typedef Eigen::Quaternion<Elem, Eigen::DontAlign> Quat;
    typedef Eigen::Matrix<Elem, 3, Eigen::Dynamic, Eigen::DontAlign> MatX;
    typedef Eigen::Map<MatX> MatXMap;
    typedef Eigen::Map<const MatX> ConstMatXMap;

    // Vertex and edge indices
    // TODO: Rename both to Index (to be used elsewhere too).
    typedef uint32_t Vertex;
    typedef uint32_t Edge;
    typedef uint32_t Index;
    // Edge cost or length
    typedef Elem Cost;

    typedef flann::Matrix<Elem> FlannMat;
    typedef flann::Index<flann::L2_3D<Elem>> FlannIndex;
    typedef std::shared_ptr<FlannIndex> FlannIndexPtr;
    typedef std::shared_ptr<const FlannIndex> ConstFlannIndexPtr;

    enum Label
    {
        TRAVERSABLE = 0,
        EMPTY = 1,
        UNKNOWN = 2,
        EDGE = 3,
        ACTOR = 4,
        OBSTACLE = 5
    };
    typedef Buffer<uint8_t> Labels;

    const Vertex INVALID_VERTEX = std::numeric_limits<Vertex>::max();

    void transform_to_pose(const geometry_msgs::Transform& tf, geometry_msgs::Pose& pose)
    {
        pose.position.x = tf.translation.x;
        pose.position.y = tf.translation.y;
        pose.position.z = tf.translation.z;
        pose.orientation = tf.rotation;
    }

    void transform_to_pose(const geometry_msgs::TransformStamped& tf, geometry_msgs::PoseStamped& pose)
    {
        pose.header = tf.header;
        transform_to_pose(tf.transform, pose.pose);
    }

    const sensor_msgs::PointField* find_field(const sensor_msgs::PointCloud2& cloud, const std::string& name)
    {
        for (const auto& f: cloud.fields)
        {
            if (f.name == name)
            {
                return &f;
            }
        }
        return nullptr;
    }

    template<typename T>
    void fill_field(const std::string& name, const T* it, sensor_msgs::PointCloud2& cloud)
    {
        size_t n = cloud.height * cloud.width;
        sensor_msgs::PointCloud2Iterator<T> field_it(cloud, name);
        const auto end = it + n;
        for (; it != end; ++it, ++field_it)
        {
            *field_it = *it;
        }
    }

    template<typename T>
    void fill_const_field(const std::string& name, const T& value, sensor_msgs::PointCloud2& cloud)
    {
        size_t n = cloud.height * cloud.width;
        sensor_msgs::PointCloud2Iterator<T> field_it(cloud, name);
        const auto end = field_it + n;
        for (; field_it != end; ++field_it)
        {
            *field_it = value;
        }
    }

    void create_debug_cloud(
            const flann::Matrix<Elem>& points,
            const flann::Matrix<Elem>& normals,
            sensor_msgs::PointCloud2& cloud)
    {
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(19,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
                "normal_x", 1, sensor_msgs::PointField::FLOAT32,
                "normal_y", 1, sensor_msgs::PointField::FLOAT32,
                "normal_z", 1, sensor_msgs::PointField::FLOAT32,
                "num_normal_pts", 1, sensor_msgs::PointField::UINT8,
                "ground_diff_std", 1, sensor_msgs::PointField::FLOAT32,
                "ground_diff_min", 1, sensor_msgs::PointField::FLOAT32,
                "ground_diff_max", 1, sensor_msgs::PointField::FLOAT32,
                "ground_abs_diff_mean", 1, sensor_msgs::PointField::FLOAT32,
                "num_obstacle_pts", 1, sensor_msgs::PointField::UINT8,
                "normal_label", 1, sensor_msgs::PointField::UINT8,
                "final_label", 1, sensor_msgs::PointField::UINT8,
                "path_cost", 1, sensor_msgs::PointField::FLOAT32,
                "utility", 1, sensor_msgs::PointField::FLOAT32,
                "final_cost", 1, sensor_msgs::PointField::FLOAT32,
                "occupied", 1, sensor_msgs::PointField::UINT16,
                "empty", 1, sensor_msgs::PointField::UINT16);
        modifier.resize(points.rows);

        sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x"), nx_it(cloud, "normal_x");
        for (size_t i = 0; i < points.rows; ++i, ++x_it, ++nx_it)
        {
            x_it[0] = points[i][0];
            x_it[1] = points[i][1];
            x_it[2] = points[i][2];
            nx_it[0] = normals[i][0];
            nx_it[1] = normals[i][1];
            nx_it[2] = normals[i][2];
        }
        fill_const_field("num_normal_pts", uint8_t(0), cloud);
        fill_const_field("ground_diff_std", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("ground_diff_min", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("ground_diff_max", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("ground_abs_diff_mean", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("num_obstacle_pts", uint8_t(0), cloud);

        fill_const_field("normal_label", uint8_t(UNKNOWN), cloud);
        fill_const_field("final_label", uint8_t(UNKNOWN), cloud);
        fill_const_field("path_cost", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("utility", std::numeric_limits<float>::quiet_NaN(), cloud);
        fill_const_field("final_cost", std::numeric_limits<float>::quiet_NaN(), cloud);

        fill_const_field("occupied", uint16_t(0), cloud);
        fill_const_field("empty", uint16_t(0), cloud);
    }

    void create_xyz_cloud(
            const flann::Matrix<Elem>& points,
            sensor_msgs::PointCloud2& cloud)
    {
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(3,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32);
        modifier.resize(points.rows);

        sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
        for (size_t i = 0; i < points.rows; ++i, ++x_it)
        {
            x_it[0] = points[i][0];
            x_it[1] = points[i][1];
            x_it[2] = points[i][2];
        }
    }

    template<typename T>
    class Query
    {
    public:
        Query(const flann::Index<flann::L2_3D<T>>& index, const flann::Matrix<T>& queries,
                const int k = 1, const T radius = std::numeric_limits<T>::infinity()):
                nn_buf_(queries.rows * k),
                dist_buf_(queries.rows * k),
                nn_(nn_buf_.begin(), queries.rows, k),
                dist_(dist_buf_.begin(), queries.rows, k)
        {
            flann::SearchParams params;
            params.cores = 0;
            if (radius < std::numeric_limits<T>::infinity())
            {
                params.max_neighbors = k;
                index.radiusSearch(queries, nn_, dist_, radius, params);
            }
            else
            {
                index.knnSearch(queries, nn_, dist_, k, params);
            }
        }
        Buffer<int> nn_buf_;
        Buffer<T> dist_buf_;
        flann::Matrix<int> nn_;
        flann::Matrix<T> dist_;
    };

    template<typename T>
    class RadiusQuery
    {
    public:
        RadiusQuery(const flann::Index<flann::L2_3D<T>>& index, const flann::Matrix<T>& queries, const T radius):
                nn_(queries.rows),
                dist_(queries.rows)
        {
            flann::SearchParams params;
            params.cores = 0;
            index.radiusSearch(queries, nn_, dist_, radius, params);
        }
        std::vector<std::vector<int>> nn_;
        std::vector<std::vector<T>> dist_;
    };

//    template<typename T>
//    Query<T> query(const flann::Index<flann::L2_3D<T>>& index, const flann::Matrix<T>& queries, int k = 1)
//    {
//        return Query<T>(index, queries, k);
//    }

    template<typename V>
    class ValueIterator
    {
    public:
        ValueIterator(const V& val):
                value_(val)
        {
        }
        ValueIterator& operator++()
        {
            value_++;
            return *this;
        }
        ValueIterator& operator--()
        {
            value_--;
            return *this;
        }
        bool operator!=(const ValueIterator<V>& other)
        {
            return value_ != other.value_;
        }
        V& operator*()
        {
            return value_;
        }
        const V& operator*() const
        {
            return value_;
        }
    private:
        V value_;
    };
    typedef ValueIterator<Vertex> VertexIter;
    typedef ValueIterator<Edge> EdgeIter;

    class Graph
    {
    public:
        Graph(flann::Matrix<Elem> points, flann::Matrix<Elem> normals,
                Buffer<uint16_t> occupied, Buffer<uint16_t> empty,
                float max_pitch, float max_roll, uint16_t empty_ratio):
                points_(points),
//                points_index_(points_, flann::KDTreeIndexParams(2)),
                points_index_(points_, flann::KDTreeSingleIndexParams()),
                normals_(normals),
                occupied_(occupied),
                empty_(empty),
                labels_(points_.rows),
                num_edge_neighbors_(points_.rows),
                max_pitch_(max_pitch),
                max_roll_(max_roll),
                empty_ratio_(empty_ratio)
        {
            // TODO: First, split points to (maybe) traversable and (hard) obstacles.
            // Traversable:
            // - approx. horizontal based on normal (with correct orientation).
            // - nearby points below ground
            // Obstacles:
            // - approx. vertical based on normal (allow opposite orientation).
//            compute_normal_labels();
        }

        bool is_empty(const uint16_t occupied, const uint16_t empty)
        {
            return empty / occupied >= empty_ratio_;
        }

        void compute_occupancy_labels(const std::vector<Elem>& robots)
        {
            Timer t;
            Index n_empty = 0;
            for (Index v = 0; v < points_.rows; ++v)
            {
                if (is_empty(occupied_[v], empty_[v]))
                {
                    labels_[v] = EMPTY;
                    ++n_empty;
                    continue;
                }
                // Avoid other robots.
                for (Index i = 0; i + 2 < robots.size(); i += 3)
                {
                    if ((ConstVec3Map(points_[v]) - ConstVec3Map(&robots[i])).norm() <= 1.5f * radius_)
                    {
//                        ROS_INFO("[%.1f, %.1f, %.1f] within %.1f m from robot [%.1f, %.1f, %.1f].",
//                                points_[v][0], points_[v][0], points_[v][0], radius_,
//                                robots[i], robots[i + 1],robots[i + 2]);
                        labels_[v] = ACTOR;
                        break;
                    }
                }
            }
            ROS_DEBUG("%u / %lu map points empty (%.3f s).",
                    n_empty, points_.rows, t.seconds_elapsed());
        }

        /** Traversability based on normal direction. */
        void compute_normal_labels()
        {
            Timer t;
            // Maximum slope allowed in some direction.
            auto max_slope = std::max(max_pitch_, max_roll_);
            auto min_z = std::cos(max_slope);
            Index n_traverable = 0, n_empty = 0, n_unknown = 0, n_edge = 0, n_actor = 0, n_obstacle = 0;
            for (size_t i = 0; i < normals_.rows; ++i)
            {
                if (labels_[i] == EMPTY)
                {
                    ++n_empty;
                }
                else if (labels_[i] == EDGE)
                {
                    ++n_edge;
                }
                else if (labels_[i] == ACTOR)
                {
                    ++n_actor;
                }
                else if (std::abs(normals_[i][2]) >= min_z)
                {
                    // Approx. horizontal based on normal (with correct orientation).
                    labels_[i] = TRAVERSABLE;
                    ++n_traverable;
                }
                else if (std::abs(normals_[i][2]) < min_z)
                {
                    // Approx. vertical based on normal (allow orientation mismatch).
                    labels_[i] = OBSTACLE;
                    ++n_obstacle;
                }
                else
                {
                    // (Currently unreachable)
                    labels_[i] = UNKNOWN;
                    ++n_unknown;
                }
            }
            ROS_DEBUG("%lu vertex labels: %u traversable, %u empty, %u unknown, %u edge, %u actor, %u obstacle (%.3f s).",
                    normals_.rows, n_traverable, n_empty, n_unknown, n_edge, n_actor, n_obstacle, t.seconds_elapsed());
        }

        void compute_graph_features(size_t min_normal_pts, Elem radius, Elem edge_min_rel_centroid_offset)
        {
            Timer t;
            const float semicircle_centroid_offset = 4. * radius_ / (3. * M_PI);
            num_normal_pts_.resize(nn_.rows);
            ground_diff_std_.resize(nn_.rows);

            size_t n_computed = 0;
            for (Vertex v0 = 0; v0 < nn_.rows; ++v0)
            {
                // Disregard empty points.
                if (labels_[v0] == EMPTY)
                {
                    continue;
                }
                Vec3 mean = Vec3::Zero();
                num_normal_pts_[v0] = 0;
                for (size_t j = 0; j < nn_.cols; ++j)
                {
                    Index v1 = nn_[v0][j];
                    // Disregard empty points.
                    if (labels_[v1] == EMPTY)
                    {
                        continue;
                    }
                    if (dist_[v0][j] <= radius)
                    {
                        mean += ConstVec3Map(points_[v1]);
                        ++num_normal_pts_[v0];
                    }
                }
//                if (n < min_normal_pts)
//                {
//                    continue;
//                }
//                mean /= n;
                mean /= num_normal_pts_[v0];
                Mat3 cov = Mat3::Zero();
                for (size_t j = 0; j < nn_.cols; ++j)
                {
                    Index v1 = nn_[v0][j];
                    // Disregard empty points.
                    if (labels_[v1] == EMPTY)
                    {
                        continue;
                    }
                    if (dist_[v0][j] <= radius)
                    {
                        Vec3 pc = (ConstVec3Map(points_[v1]) - mean);
                        cov += pc * pc.transpose();
                    }
                }
//                cov /= (n + 1);
//                cov /= (num_normal_pts_[v0] + 1);
                cov /= num_normal_pts_[v0];
                Eigen::SelfAdjointEigenSolver<Mat3> solver(cov);
                // solver.eigenvalues();
                Vec3Map normal(normals_[v0]);
                normal = solver.eigenvectors().col(0);
                ground_diff_std_[v0] = std::sqrt(solver.eigenvalues()(0));
//                ground_diff_min_
//                ++n_computed;
                // Inject support label here where we have computed mean.
//                labels_
                if ((mean - ConstVec3Map(points_[v0])).norm()
                        > edge_min_rel_centroid_offset * semicircle_centroid_offset)
                {
                    labels_[v0] = EDGE;
                }
            }
            ROS_DEBUG("Normals recomputed for %lu points from %lu nn within %.2g m: %.3f s.",
                    nn_.rows, nn_.cols, radius_, t.seconds_elapsed());
        }

        /** Traversability based on NN graph. */
        void compute_final_labels(Elem max_nn_height_diff_, Elem clearance_low, Elem clearance_high,
                Vertex min_points_obstacle, Elem max_ground_diff_std, Elem max_ground_abs_diff_mean,
                Elem min_dist_to_obstacle)
        {
            Timer t;
            ground_diff_min_.resize(nn_.rows);
            ground_diff_max_.resize(nn_.rows);
            ground_abs_diff_mean_.resize(nn_.rows);
            num_obstacle_pts_.resize(nn_.rows);
            // Maximum slope allowed in some direction.
            Index n_traverable = 0, n_empty = 0, n_unknown = 0, n_edge = 0, n_actor = 0, n_obstacle = 0;
//            ROS_INFO("NN rows: %lu, cols %lu", nn_.rows, nn_.cols);
            for (Vertex v0 = 0; v0 < nn_.rows; ++v0)
            {
                num_edge_neighbors_[v0] = 0;
                for (Vertex j = 0; j < nn_.cols; ++j)
                {
                    const auto v1 = nn_[v0][j];
                    if (labels_[v1] == EDGE)
                    {
                        ++num_edge_neighbors_[v0];
                    }
                }
                // Disregard empty points.
                if (labels_[v0] == EMPTY)
                {
                    ++n_empty;
                    continue;
                }
                else if (labels_[v0] == EDGE)
                {
                    ++n_edge;
                    continue;
                }
                else if (labels_[v0] == ACTOR)
                {
                    ++n_actor;
                    continue;
                }
                // Adjust only traversable points.
//                if (labels_[v0] != TRAVERSABLE)
//                {
//                    continue;
//                }
                // Compute ground features for all points.
//                Elem min_height_diff = std::numeric_limits<Elem>::infinity();
//                Elem max_height_diff = -std::numeric_limits<Elem>::infinity();
                ground_diff_min_[v0] = std::numeric_limits<Elem>::infinity();
                ground_diff_max_[v0] = -std::numeric_limits<Elem>::infinity();
                ground_abs_diff_mean_[v0] = 0.;
                uint8_t n = 0;
//                uint8_t n_points_obstacle;
                num_obstacle_pts_[v0] = 0;
                for (size_t j = 0; j < nn_.cols; ++j)
                {
                    const auto v1 = nn_[v0][j];
                    // Disregard empty points.
                    if (labels_[v1] == EMPTY)
                    {
                        continue;
                    }
                    // Avoid driving near obstacles.
                    if (labels_[v0] == TRAVERSABLE && labels_[v1] == OBSTACLE && dist_[v0][j] <= min_dist_to_obstacle)
                    {
                        labels_[v0] = UNKNOWN;
                        ++n_unknown;
//                        break;
                    }
                    Elem height_diff = Vec3Map(normals_[v0]).dot(Vec3Map(points_[v1]) - Vec3Map(points_[v0]));
                    Vec3 ground_pt = Vec3Map(points_[v1]) - height_diff * Vec3Map(normals_[v0]);
                    Elem ground_dist = (ground_pt - Vec3Map(points_[v0])).norm();

                    if (ground_dist <= radius_)
                    {
                        if (height_diff < ground_diff_min_[v0])
                        {
                            ground_diff_min_[v0] = height_diff;
                        }
                        if (height_diff > ground_diff_max_[v0])
                        {
                            ground_diff_max_[v0] = height_diff;
                        }
                        ground_abs_diff_mean_[v0] += std::abs(height_diff);
                        ++n;
                        if (height_diff >= clearance_low && height_diff <= clearance_high)
                        {
                            ++num_obstacle_pts_[v0];
                        }
                    }
//                    ROS_INFO("height diff.: %.2f m, ground dist.: %.2f m", height_diff, ground_dist);
//                    if (ground_dist > radius_)
//                    {
//                        continue;
//                    }
//                    if (std::abs(height_diff) > max_nn_height_diff)
//                    if (max_height_diff - min_height_diff > max_nn_height_diff)
//                    {
//                        labels_[v0] = UNKNOWN;
//                        ++n_adjusted;
//                        break;
//                    }
                }
                ground_abs_diff_mean_[v0] /= n;
                if (labels_[v0] == TRAVERSABLE
                        && (num_obstacle_pts_[v0] >= min_points_obstacle
                            || ground_diff_max_[v0] - ground_diff_min_[v0] > max_nn_height_diff_
                            || ground_diff_std_[v0] > max_ground_diff_std
                            || ground_abs_diff_mean_[v0] > max_ground_abs_diff_mean))
                {
                    labels_[v0] = UNKNOWN;
                    ++n_unknown;
                }
                else if (labels_[v0] == TRAVERSABLE)
                {
                    ++n_traverable;
                }
                else if (labels_[v0] == OBSTACLE)
                {
                    ++n_obstacle;
                }
            }
            ROS_INFO("%lu final labels: %u traversable, %u empty, %u unknown, %u edge, %u actor, %u obstacle (%.3f s).",
                    normals_.rows, n_traverable, n_empty, n_unknown, n_edge, n_actor, n_obstacle, t.seconds_elapsed());
        }

        void build_index()
        {
            Timer t;
            points_index_.buildIndex();
            ROS_DEBUG("Building index for %lu pts: %.3f s.", points_.rows, t.seconds_elapsed());
        }

        void compute_graph(Vertex k, Elem radius)
        {
            Timer t;
            nn_buf_.resize(num_vertices() * k);
            dist_buf_.resize(num_vertices() * k);
            dist_ = flann::Matrix<Elem>(dist_buf_.begin(), num_vertices(), k);
            nn_ = flann::Matrix<int>(nn_buf_.begin(), num_vertices(), k);
            t.reset();
            flann::SearchParams params;
            params.checks = 64;
            params.cores = 0;
//            params.max_neighbors = k;
//            points_index_.radiusSearch(points_, nn_, dist_, radius, params);
            points_index_.knnSearch(points_, nn_, dist_, k, params);
            k_ = k;
            radius_ = radius;
            ROS_DEBUG("NN graph (%lu pts): %.3f s.", points_.rows, t.seconds_elapsed());
        }

        inline Vertex num_vertices() const
        {
            return points_.rows;
        }
        inline Edge num_edges() const
        {
            return nn_.cols;
        }
        inline std::pair<VertexIter, VertexIter> vertices() const
        {
            return { 0, num_vertices() };
        }

        inline std::pair<EdgeIter, EdgeIter> out_edges(const Vertex& u) const
        {
            // TODO: Limit to valid edges here or just by costs?
            return { u * num_edges(), (u + 1) * num_edges() };
        }
        inline Edge out_degree(const Vertex& u) const
        {
            return num_edges();
        }
        inline Vertex source(const Edge& e) const
        {
            return e / num_edges();
        }
        inline Vertex target_index(const Edge& e) const
        {
            return e % num_edges();
        }
        inline Vertex target(const Edge& e) const
        {
            return nn_[source(e)][target_index(e)];
        }
        inline Cost cost(const Edge& e) const
        {
            const auto v0 = source(e);
            const auto v1_index = target_index(e);
            const auto v1 = target(e);
//            if (labels_[v0] != TRAVERSABLE || labels_[v1] != TRAVERSABLE)
            if (labels_[v1] != TRAVERSABLE)
            {
                return std::numeric_limits<Cost>::infinity();
            }
            Cost d = std::sqrt(dist_[v0][v1_index]);
            if (d > radius_)
            {
                return std::numeric_limits<Cost>::infinity();
            }
            Elem height_diff = points_[v1][2] - points_[v0][2];
            Elem inclination = std::asin(std::abs(height_diff) / d);
            if (inclination > max_pitch_)
            {
                return std::numeric_limits<Cost>::infinity();
            }
            // TODO: Check pitch and roll separately.
            /*
            Elem pitch0 = std::acos(normals_[v0][2]);
            if (pitch0 > max_pitch_)
            {
                return std::numeric_limits<Cost>::infinity();
            }
            Elem pitch1 = std::acos(normals_[v1][2]);
            if (pitch1 > max_pitch_)
            {
                return std::numeric_limits<Cost>::infinity();
            }
            float roll0 = 0.f;
            float roll1 = 0.f;
            */
             // Initialize with distance computed in NN search.
            // Multiple with relative pitch and roll.
//            d *= (1.f + (pitch0 + pitch1 + inclination) / 3.f / max_pitch_ + (roll0 + roll1) / 2.f / max_roll_);
            d *= (1.f + (inclination / max_pitch_));
//            std::cout << v0 << " -> " << v1 << ": " << d << std::endl;
            return d;
        }

        flann::Matrix<Elem> points_;
        flann::Index<flann::L2_3D<Elem>> points_index_;
        flann::Matrix<Elem> normals_;
        Buffer<uint16_t> occupied_;
        Buffer<uint16_t> empty_;
        Buffer<uint8_t> num_normal_pts_;
        // from ball neighborhood
        Buffer<Elem> ground_diff_std_;
        // circle in ground plane
        Buffer<Elem> ground_diff_min_;
        Buffer<Elem> ground_diff_max_;
        Buffer<Elem> ground_abs_diff_mean_;
        Buffer<uint8_t> num_obstacle_pts_;
        Buffer<uint8_t> labels_;
        Buffer<uint8_t> num_edge_neighbors_;

        // NN and distances
        int k_;
        Elem radius_;
        Buffer<int> nn_buf_;
        Buffer<Elem> dist_buf_;
        flann::Matrix<int> nn_;
        flann::Matrix<Elem> dist_;

        float max_pitch_;
        float max_roll_;
        uint16_t empty_ratio_;
    };

    class EdgeCosts
    {
    public:
        EdgeCosts(const Graph& g)
                :
                g_(g)
        {
        }
        inline Cost operator[](const Edge& e) const
        {
            return g_.cost(e);
        }
    private:
        const Graph& g_;
    };

}  // namespace naex

using namespace naex;

namespace boost
{
    template<>
    struct graph_traits<naex::Graph>
    {
        typedef Vertex vertex_descriptor;
        typedef Vertex vertices_size_type;
        typedef Edge edge_descriptor;
        typedef Edge edges_size_type;

        typedef directed_tag directed_category;
        // typedef undirected_tag directed_category;
        // typedef allow_parallel_edge_tag edge_parallel_category;
        typedef disallow_parallel_edge_tag edge_parallel_category;

        typedef bidirectional_traversal_tag traversal_category;
        typedef VertexIter vertex_iterator;
        typedef EdgeIter out_edge_iterator;
    };

    inline Vertex num_vertices(const Graph& g)
    {
        return g.num_vertices();
    }

    inline std::pair<VertexIter, VertexIter> vertices(const Graph& g)
    {
        return g.vertices();
    }

    inline Vertex source(Edge e, const Graph& g)
    {
        return g.source(e);
    }

    inline Vertex target(Edge e, const Graph& g)
    {
        return g.target(e);
    }

    inline std::pair<EdgeIter, EdgeIter> out_edges(Vertex u, const Graph& g)
    {
        return g.out_edges(u);
    }

    inline Edge out_degree(Vertex u, const Graph& g)
    {
        return g.out_degree(u);
    }

    template<>
    class property_traits<EdgeCosts>
    {
    public:
        typedef Edge key_type;
        typedef Cost value_type;
        typedef readable_property_map_tag category;
    };

    inline Cost get(const EdgeCosts& map, const Edge& key)
    {
        return map[key];
    }

}  // namespace boost

// Include dijkstra header once all used concepts are defined.
// https://groups.google.com/g/boost-developers-archive/c/G2qArovLKzk
// #include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/graph/dijkstra_shortest_paths_no_color_map.hpp>

namespace naex
{
    class Map
    {
        static const size_t DEFAULT_CAPACITY = 10000000;
    public:
        Map():
                points_buf_(3 * DEFAULT_CAPACITY),
                normals_buf_(3 * DEFAULT_CAPACITY),
                viewpoints_buf_(3 * DEFAULT_CAPACITY),
                occupied_buf_(DEFAULT_CAPACITY),
                empty_buf_(DEFAULT_CAPACITY),
                points_(points_buf_.begin(), 0, 3),
                normals_(normals_buf_.begin(), 0, 3),
                viewpoints_(viewpoints_buf_.begin(), 0, 3),
                index_(),
                points_min_dist_(0.2),
                min_empty_cos_(0.3)
        {
            std::fill(points_buf_.begin(), points_buf_.end(), 0.);
            std::fill(normals_buf_.begin(), normals_buf_.end(), 0.);
            std::fill(viewpoints_buf_.begin(), viewpoints_buf_.end(), 0.);
            std::fill(occupied_buf_.begin(), occupied_buf_.end(), 1);
            std::fill(empty_buf_.begin(), empty_buf_.end(), 0);
        }

        void update_matrix_wrappers(size_t n)
        {
            Lock lock(snapshot_mutex_);
            points_ = flann::Matrix<Elem>(points_buf_.begin(), n, 3);
            normals_ = flann::Matrix<Elem>(normals_buf_.begin(), n, 3);
            viewpoints_ = flann::Matrix<Elem>(viewpoints_buf_.begin(), n, 3);
        }

        // TODO: Snapshot whole map object.
        void snapshot(FlannMat& points, FlannMat& normals, FlannMat& viewpoints, FlannIndexPtr& index,
                Buffer<uint16_t>& occupied, Buffer<uint16_t>& empty)
        {
            Lock lock(snapshot_mutex_);
            points = points_;
            normals = normals_;
            viewpoints = viewpoints_;
            index = index_;
            occupied = occupied_buf_;
            empty = empty_buf_;
            ROS_INFO("Original size %lu, snapshot size %lu.", points_.rows, points.rows);
        }

        void snapshot(flann::Matrix<Elem>& points)
        {
            Lock lock(snapshot_mutex_);
            points = points_;
        }

        void update_index()
        {
            Timer t;
            Lock lock(snapshot_mutex_);
            if (!empty())
            {
                index_ = std::make_shared<flann::Index<flann::L2_3D<Elem>>>(points_, flann::KDTreeSingleIndexParams());
                index_->buildIndex();
            }
            ROS_DEBUG("Updating index: %.3f s.", t.seconds_elapsed());
        }

        /** Resize point buffers if necessary, update wrappers and index. */
        void reserve(size_t n)
        {
            Timer t;
            if (n < capacity())
            {
                return;
            }
            points_buf_ = points_buf_.copy(3 * n);
            normals_buf_ = normals_buf_.copy(3 * n);
            viewpoints_buf_ = viewpoints_buf_.copy(3 * n);
            update_matrix_wrappers(n);
            update_index();
            ROS_INFO("Capacity increased to %lu points: %.3f s.", n, t.seconds_elapsed());
        }

        Elem* end()
        {
            return points_buf_.begin() + 3 * size();
        }

//        void merge(flann::Matrix<Elem> points, flann::Matrix<Elem> origin)
        void merge(const flann::Matrix<Elem>& points, const flann::Matrix<Elem>& origin)
        {
            Lock lock(snapshot_mutex_);
            Timer t;
            ROS_DEBUG("Merging cloud started. Capacity %lu points.", capacity());
            // TODO: Forbid allocation while in use or lock?
            reserve(size() + points.rows);

            if (empty())
            {
                auto it_points = end();
                for (size_t i = 0; i < points.rows; ++i)
                {
                    *it_points++ = points[i][0];
                    *it_points++ = points[i][1];
                    *it_points++ = points[i][2];
                }
                update_matrix_wrappers(points.rows);
                update_index();
                ROS_INFO("Map initialized with %lu points (%.3f s).", points.rows, t.seconds_elapsed());
                return;
            }

            // TODO: Update static (hit) / dynamic (see through) points.
            Timer t_dyn;
            // Compute input points directions.
            ConstVec3Map x_origin(origin[0]);
            Buffer<Elem> dirs_buf(points.rows * points.cols);
            FlannMat dirs(dirs_buf.begin(), points.rows, points.cols);
            for (Index i = 0; i < points.rows; ++i)
            {
                Vec3Map dir(dirs[i]);
                dir = ConstVec3Map(points[i]) - x_origin;
                const Elem norm = dir.norm();
                if (std::isnan(norm) || std::isinf(norm) || norm < 1e-3)
                {
                    ROS_INFO_THROTTLE(1.0, "Could not normalize direction [%.1g, %.1g, %.1g], norm %.1g.",
                            dir.x(), dir.y(), dir.z(), norm);
                    dir = Vec3::Zero();
                }
                else
                {
                    dir /= norm;
                }
            }
            // Create index of view directions.
            FlannIndex dirs_index(dirs, flann::KDTreeSingleIndexParams());
            dirs_index.buildIndex();

            // Get map points nearby to check.
            Elem nearby_dist = 25.;
            RadiusQuery<Elem> q_nearby(*index_, origin, nearby_dist);
            Index n_nearby = q_nearby.nn_[0].size();
            ROS_DEBUG("Found %u points up to %.1f m from sensor (%.3f s).",
                    n_nearby, nearby_dist, t_dyn.seconds_elapsed());
//            Buffer<Index> occupied_buf(n_nearby);
//            Buffer<Index> empty_buf(n_nearby);
            Buffer<Elem> nearby_dirs_buf(3 * n_nearby);
            FlannMat nearby_dirs(nearby_dirs_buf.begin(), n_nearby, 3);
            for (Index i = 0; i < n_nearby; ++i)
            {
                const Index v0 = q_nearby.nn_[0][i];
                Vec3Map dir(nearby_dirs[i]);
                dir = ConstVec3Map(points_[v0]) - x_origin;
                const Elem norm = dir.norm();
                if (std::isnan(norm) || std::isinf(norm) || norm < 1e-3)
                {
                    ROS_INFO_THROTTLE(1.0, "Could not normalize direction [%.1g, %.1g, %.1g], norm %.1g.",
                            dir.x(), dir.y(), dir.z(), norm);
                    dir = Vec3::Zero();
                }
                else
                {
                    dir /= norm;
                }
            }
            int k_dirs = 5;  // Number of closest rays to check.
            uint16_t max_observations = 15;
            Query<Elem> q_dirs(dirs_index, nearby_dirs, k_dirs);
            for (Index i = 0; i < n_nearby; ++i)
            {
                const Index v_map = q_nearby.nn_[0][i];
                ConstVec3Map x_map(points_[v_map]);
                uint16_t empty_orig = empty_buf_[v_map];
                for (Index j = 0; j < k_dirs; ++j)
                {
                    const Index v_new = q_dirs.nn_[i][j];
                    // TODO: Avoid segfault v_new > size?
                    // Discarding small clouds seems to do the trick?
                    if (v_new >= points.rows)
                    {
                        ROS_WARN_THROTTLE(1.0, "Skipping invalid NN (%u >= %lu) during merge.", v_new, points.rows);
                        continue;
                    }
                    ConstVec3Map x_new(points[v_new]);
                    // Match close neighbors as a single static object.
                    const Elem d = (x_new - x_map).norm();
                    if (d <= points_min_dist_)
                    {
                        // Static / occupied.
                        if (occupied_buf_[v_map] >= max_observations)
                        {
                            occupied_buf_[v_map] /= 2;
                            empty_buf_[v_map] /= 2;
                        }
                        ++occupied_buf_[v_map];
                        // Clear empty marks from this cloud.
                        empty_buf_[v_map] = empty_orig;
                        break;
                    }
                    // Discard rays far from the map points.
                    const Elem d_new = (x_new - x_origin).norm();
                    if (std::isnan(d_new) || std::isinf(d_new) || d_new < 1e-3)
                    {
                        ROS_WARN_THROTTLE(1.0, "Discarding invalid point [%.1g, %.1g, %.1g], distance to origin %.1g.",
                                x_new.x(), x_new.y(), x_new.z(), d_new);
                        continue;
                    }
                    const Elem line_dist = (x_new - x_origin).cross(x_origin - x_map).norm() / d_new;
                    if (line_dist / 2. > points_min_dist_)
                    {
                        continue;
                    }
                    // Does farther point see through the map surface?
                    const Elem d_map = (x_map - x_origin).norm();
                    // TODO: To be usable as this normal must have correct orientation.
                    ConstVec3Map n_map(normals_[v_map]);
//                    Elem level = n_map.dot(x_new - x_map);
//                    if (d_new > d_map && level < 0.)
                    Elem cos = n_map.dot(ConstVec3Map(dirs[v_new]));
                    if (d_new > d_map && std::abs(cos) > min_empty_cos_)
                    {
                        // Dynamic / see through.
                        if (occupied_buf_[v_map] >= max_observations)
                        {
                            occupied_buf_[v_map] /= 2;
                            empty_buf_[v_map] /= 2;
                        }
                        ++empty_buf_[v_map];
                    }
                }
//                ROS_INFO("Point [%.1f, %.1f, %.1f]: %u occupied, %u empty.",
//                        x_map.x(), x_map.y(), x_map.z(), occupied_buf_[v_map], empty_buf_[v_map]);
                if (empty_buf_[v_map] / occupied_buf_[v_map] >= 4)
                {
                    // TODO: Remove point?
                    // Handle this in processing NN.
                }
            }
            ROS_DEBUG("Checking and updating %u static/dynamic points nearby: %.3f s.",
                    n_nearby, t_dyn.seconds_elapsed());

            // Find NN distance within current map.
            t.reset();
            Query<Elem> q(*index_, points, 1);
            ROS_DEBUG("Input cloud NN searched finished: %.3f s.", t.seconds_elapsed());

            // Merge points with distance to NN higher than threshold.
            // We'll assume that input points also (approx.) comply to the
            // same min. distance threshold, so each added point can be tested
            // separately.
            t.reset();
            size_t n_added = 0;
            auto it_points = end();
//            Elem mean = 0.;
            Elem min_dist_2 = points_min_dist_ * points_min_dist_;
            for (size_t i = 0; i < points.rows; ++i)
            {
//                auto dist = std::sqrt(q.dist_[i][0]);
//                mean += std::sqrt(q.dist_[i][0]);
//                ROS_INFO("%lu: %.3f >= %.3f?", i, dist, points_min_dist_);
//                if (dist >= points_min_dist_)
                if (q.dist_[i][0] >= min_dist_2)
                {
                    *it_points++ = points[i][0];
                    *it_points++ = points[i][1];
                    *it_points++ = points[i][2];
                    ++n_added;
                }
            }
//            mean /= points.rows;
//            ROS_INFO("Mean distance to map points: %.3f m.", mean);
            size_t n_map = size();
            update_matrix_wrappers(size() + n_added);
            // TODO: flann::Index::addPoints (what indices? what with removed indices?)
            update_index();
            ROS_INFO("%lu points merged into map with %lu points (%.3f s).", n_added, n_map, t.seconds_elapsed());
        }

        void create_cloud(sensor_msgs::PointCloud2& cloud)
        {
            sensor_msgs::PointCloud2Modifier modifier(cloud);
            modifier.setPointCloud2Fields(9,
                    "x", 1, sensor_msgs::PointField::FLOAT32,
                    "y", 1, sensor_msgs::PointField::FLOAT32,
                    "z", 1, sensor_msgs::PointField::FLOAT32,
                    "normal_x", 1, sensor_msgs::PointField::FLOAT32,
                    "normal_y", 1, sensor_msgs::PointField::FLOAT32,
                    "normal_z", 1, sensor_msgs::PointField::FLOAT32,
                    "occupied", 1, sensor_msgs::PointField::UINT16,
                    "empty", 1, sensor_msgs::PointField::UINT16,
                    "dynamic", 1, sensor_msgs::PointField::FLOAT32);
            FlannMat points;
            snapshot(points);
            modifier.resize(points.rows);

            sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> nx_it(cloud, "normal_x");
            sensor_msgs::PointCloud2Iterator<uint16_t> occupied_it(cloud, "occupied");
            sensor_msgs::PointCloud2Iterator<uint16_t> empty_it(cloud, "empty");
            sensor_msgs::PointCloud2Iterator<float> dynamic_it(cloud, "dynamic");
            for (size_t i = 0; i < points.rows; ++i, ++x_it, ++nx_it, ++occupied_it, ++empty_it, ++dynamic_it)
            {
                x_it[0] = points[i][0];
                x_it[1] = points[i][1];
                x_it[2] = points[i][2];
                nx_it[0] = normals_[i][0];
                nx_it[1] = normals_[i][1];
                nx_it[2] = normals_[i][2];
                *occupied_it = occupied_buf_[i];
                *empty_it = empty_buf_[i];
                *dynamic_it = float(empty_buf_[i]) / occupied_buf_[i];
            }
        }

        float points_min_dist_;
        float min_empty_cos_;

    protected:
        size_t capacity() const
        {
            return points_buf_.size() / 3;
        }

        size_t size() const
        {
            // TODO: Lock? Make thread safe?
            return points_.rows;
        }

        size_t empty() const
        {
            return size() == 0;
        }

        Buffer<Elem> points_buf_;
        Buffer<Elem> normals_buf_;
        Buffer<Elem> viewpoints_buf_;
        Buffer<uint16_t> occupied_buf_;
        Buffer<uint16_t> empty_buf_;

        typedef std::recursive_mutex Mutex;
        typedef std::lock_guard<Mutex> Lock;
        Mutex snapshot_mutex_;
        flann::Matrix<Elem> points_;
        flann::Matrix<Elem> normals_;
        flann::Matrix<Elem> viewpoints_;
        std::shared_ptr<flann::Index<flann::L2_3D<Elem>>> index_;
    };

    class Planner
    {
    public:
        Planner(ros::NodeHandle& nh, ros::NodeHandle& pnh):
                nh_(nh),
                pnh_(pnh),
                tf_(),
                position_name_("x"),
                normal_name_("normal_x"),
                map_frame_(""),
                robot_frame_("base_footprint"),
                robot_frames_(),
                max_cloud_age_(5.),
                max_pitch_(30. / 180. * M_PI),
                max_roll_(30. / 180. * M_PI),
                empty_ratio_(2),
                filter_robots_(false),
                neighborhood_knn_(12),
                neighborhood_radius_(.5),
                min_normal_pts_(9),
                normal_radius_(0.5),
                max_nn_height_diff_(0.15),
                viewpoints_update_freq_(1.),
                viewpoints_(),
                clearance_low_(0.15),
                clearance_high_(0.8),
                min_points_obstacle_(3),
                max_ground_diff_std_(0.1),
                max_ground_abs_diff_mean_(0.1),
                edge_min_centroid_offset_(0.75),
                min_dist_to_obstacle_(0.7),
                other_viewpoints_(),
                min_vp_distance_(1.5),
                max_vp_distance_(5.),
                self_factor_(0.25),
                planning_freq_(0.5),
                queue_size_(5.)
        {
            // Invalid position invokes exploration mode.
            last_request_.start.pose.position.x = std::numeric_limits<double>::quiet_NaN();
            last_request_.start.pose.position.y = std::numeric_limits<double>::quiet_NaN();
            last_request_.start.pose.position.z = std::numeric_limits<double>::quiet_NaN();
            last_request_.goal.pose.position.x = std::numeric_limits<double>::quiet_NaN();
            last_request_.goal.pose.position.y = std::numeric_limits<double>::quiet_NaN();
            last_request_.goal.pose.position.z = std::numeric_limits<double>::quiet_NaN();
            last_request_.tolerance = 32.;
            configure();
        }

        void update_params(const ros::WallTimerEvent& evt)
        {
            pnh_.param("max_nn_height_diff", max_nn_height_diff_, max_nn_height_diff_);
            pnh_.param("clearance_low", clearance_low_, clearance_low_);
            pnh_.param("clearance_high", clearance_high_, clearance_high_);
            pnh_.param("min_points_obstacle", min_points_obstacle_, min_points_obstacle_);
            pnh_.param("max_ground_diff_std", max_ground_diff_std_, max_ground_diff_std_);
            pnh_.param("max_ground_abs_diff_mean", max_ground_abs_diff_mean_, max_ground_abs_diff_mean_);
            pnh_.param("edge_min_centroid_offset", edge_min_centroid_offset_, edge_min_centroid_offset_);
            pnh_.param("min_dist_to_obstacle", min_dist_to_obstacle_, min_dist_to_obstacle_);
        }

        void configure()
        {
            pnh_.param("position_name", position_name_, position_name_);
            pnh_.param("normal_name", normal_name_, normal_name_);
            pnh_.param("map_frame", map_frame_, map_frame_);
            pnh_.param("robot_frame", robot_frame_, robot_frame_);
            pnh_.param("robot_frames", robot_frames_, robot_frames_);
            pnh_.param("max_cloud_age", max_cloud_age_, max_cloud_age_);
            pnh_.param("max_pitch", max_pitch_, max_pitch_);
            pnh_.param("max_roll", max_roll_, max_roll_);
            pnh_.param("neighborhood_knn", neighborhood_knn_, neighborhood_knn_);
            pnh_.param("neighborhood_radius", neighborhood_radius_, neighborhood_radius_);
            pnh_.param("min_normal_pts", min_normal_pts_, min_normal_pts_);
            pnh_.param("normal_radius", normal_radius_, normal_radius_);

            pnh_.param("max_nn_height_diff", max_nn_height_diff_, max_nn_height_diff_);
            pnh_.param("clearance_low", clearance_low_, clearance_low_);
            pnh_.param("clearance_high", clearance_high_, clearance_high_);
            pnh_.param("min_points_obstacle", min_points_obstacle_, min_points_obstacle_);
            pnh_.param("max_ground_diff_std", max_ground_diff_std_, max_ground_diff_std_);
            pnh_.param("max_ground_abs_diff_mean", max_ground_abs_diff_mean_, max_ground_abs_diff_mean_);
            pnh_.param("edge_min_centroid_offset", edge_min_centroid_offset_, edge_min_centroid_offset_);
            pnh_.param("min_dist_to_obstacle", min_dist_to_obstacle_, min_dist_to_obstacle_);

            pnh_.param("viewpoints_update_freq", viewpoints_update_freq_, viewpoints_update_freq_);
            pnh_.param("min_vp_distance", min_vp_distance_, min_vp_distance_);
            pnh_.param("max_vp_distance", max_vp_distance_, max_vp_distance_);
            pnh_.param("self_factor", self_factor_, self_factor_);
            pnh_.param("planning_freq", planning_freq_, planning_freq_);

            int num_input_clouds = 1;
            pnh_.param("num_input_clouds", num_input_clouds, num_input_clouds);
            pnh_.param("input_queue_size", queue_size_, queue_size_);
            pnh_.param("points_min_dist", map_.points_min_dist_, map_.points_min_dist_);
            pnh_.param("min_empty_cos", map_.min_empty_cos_, map_.min_empty_cos_);
            pnh_.param("empty_ratio", empty_ratio_, empty_ratio_);
            pnh_.param("filter_robots", filter_robots_, filter_robots_);

            bool among_robots = false;
            for (const auto& kv: robot_frames_)
            {
                if (kv.second == robot_frame_)
                {
                    among_robots = true;
                }
            }
            if (!among_robots)
            {
                ROS_INFO("Inserting robot frame among all robot frames.");
                robot_frames_["SELF"] = robot_frame_;
            }

            viewpoints_.reserve(size_t(7200. * viewpoints_update_freq_) * 3);
            other_viewpoints_.reserve(size_t(7200. * viewpoints_update_freq_) * 3 * robot_frames_.size());

            // C++14
//            tf_ = std::make_unique<tf2_ros::Buffer>(ros::Duration(10.));
//            tf_sub_ = std::make_unique<tf2_ros::TransformListener>(*tf_);
            tf_.reset(new tf2_ros::Buffer(ros::Duration(30.)));
            tf_sub_.reset(new tf2_ros::TransformListener(*tf_));

            normal_label_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("normal_label_cloud", 5);
            final_label_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("final_label_cloud", 5);
            path_cost_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("path_cost_cloud", 5);
            utility_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("utility_cloud", 5);
            final_cost_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("final_cost_cloud", 5);

            path_pub_ = nh_.advertise<nav_msgs::Path>("path", 5);
            minpos_path_pub_ = nh_.advertise<nav_msgs::Path>("minpos_path", 5);

            viewpoints_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("viewpoints", 5);
            other_viewpoints_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("other_viewpoints", 5);
            map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 5);

            cloud_sub_ = nh_.subscribe("input_map", queue_size_, &Planner::cloud_received, this);
            for (int i = 0; i < num_input_clouds; ++i)
            {
                std::stringstream ss;
                ss << "input_cloud_" << i;
                auto sub = nh_.subscribe(ss.str(), queue_size_, &Planner::input_cloud_received, this);
                input_cloud_subs_.push_back(sub);
            }

            viewpoints_update_timer_ =  nh_.createTimer(ros::Rate(viewpoints_update_freq_),
                    &Planner::gather_viewpoints, this);
            planning_timer_ =  nh_.createTimer(ros::Rate(planning_freq_),
                    &Planner::planning_timer_cb, this);
            update_params_timer_ = nh_.createWallTimer(ros::WallDuration(2.0),
                    &Planner::update_params, this);

            get_plan_service_ = nh_.advertiseService("get_plan", &Planner::plan, this);
        }

        void gather_viewpoints(const ros::TimerEvent& event)
        {
            ROS_DEBUG("Gathering viewpoints.");
            if (map_frame_.empty())
            {
                ROS_ERROR("Could not gather robot positions due to missing map frame.");
                return;
            }
            Lock lock(viewpoints_mutex_);
            for (const auto& kv: robot_frames_)
            {
                const auto& frame = kv.second;
                try
                {
                    // Get last transform available (don't wait).
                    auto tf = tf_->lookupTransform(map_frame_, frame, ros::Time());
                    if (frame == robot_frame_)
                    {
                        viewpoints_.push_back(tf.transform.translation.x);
                        viewpoints_.push_back(tf.transform.translation.y);
                        viewpoints_.push_back(tf.transform.translation.z);
                    }
                    else
                    {
                        other_viewpoints_.push_back(tf.transform.translation.x);
                        other_viewpoints_.push_back(tf.transform.translation.y);
                        other_viewpoints_.push_back(tf.transform.translation.z);
                    }
                }
                catch (const tf2::TransformException& ex)
                {
                    ROS_WARN("Could not get robot %s position: %s.", frame.c_str(), ex.what());
                    continue;
                }
            }
            auto now = ros::Time::now();
            sensor_msgs::PointCloud2 vp_cloud;
            flann::Matrix<Elem> vp(viewpoints_.data(), viewpoints_.size(), 3);
            create_xyz_cloud(vp, vp_cloud);
            vp_cloud.header.frame_id = map_frame_;
            vp_cloud.header.stamp = now;
            viewpoints_pub_.publish(vp_cloud);

            sensor_msgs::PointCloud2 other_vp_cloud;
            flann::Matrix<Elem> other_vp(other_viewpoints_.data(), other_viewpoints_.size(), 3);
            create_xyz_cloud(other_vp, other_vp_cloud);
            other_vp_cloud.header.frame_id = map_frame_;
            other_vp_cloud.header.stamp = now;
            other_viewpoints_pub_.publish(other_vp_cloud);
        }

        void trace_path_indices(Vertex start, Vertex goal, const Buffer<Vertex>& predecessor,
                std::vector<Vertex>& path_indices)
        {
            assert(predecessor[start] == start);
            Vertex v = goal;
            while (v != start)
            {
                path_indices.push_back(v);
                v = predecessor[v];
            }
            path_indices.push_back(v);
            std::reverse(path_indices.begin(), path_indices.end());
        }

        void append_path(const std::vector<Vertex>& path_indices,
                const flann::Matrix<Elem>& points,
                const flann::Matrix<Elem>& normals,
                nav_msgs::Path& path)
        {
            if (path_indices.empty())
            {
                return;
            }
            path.poses.reserve(path.poses.size() + path_indices.size());
            for (const auto& v: path_indices)
            {
                geometry_msgs::PoseStamped pose;
                pose.pose.position.x = points[v][0];
                pose.pose.position.y = points[v][1];
                pose.pose.position.z = points[v][2];
                pose.pose.orientation.w = 1.;
                if (!path.poses.empty())
                {
                    Vec3 x(pose.pose.position.x - path.poses.back().pose.position.x,
                            pose.pose.position.y - path.poses.back().pose.position.y,
                            pose.pose.position.z - path.poses.back().pose.position.z);
                    x.normalize();
                    Vec3Map z(normals[v]);
                    // Fix z direction to be consistent with the previous pose.
                    // As we start from the current robot pose with correct z
                    // orientation, all following z directions get corrected.
//                    Quat q_prev(path.poses.back().pose.orientation.w,
//                            path.poses.back().pose.orientation.x,
//                            path.poses.back().pose.orientation.y,
//                            path.poses.back().pose.orientation.z);
//                    Mat3 R_prev;
//                    R_prev = q_prev;
//                    Vec3 z_prev = R_prev.col(2);
//                    if (z.dot(z_prev) < 0.)
//                    {
//                        z = -z;
//                    }
                    // Assume map z points upward.
                    if (z.dot(Vec3(0.f, 0.f, 1.f)) < 0.)
                    {
                        z = -z;
                    }

                    Mat3 m;
//                    m.row(0) = x;
//                    m.row(1) = z.cross(x);
//                    m.row(2) = z;
                    m.col(0) = x;
                    m.col(1) = z.cross(x);
                    m.col(2) = z;
                    Quat q;
                    q = m;
                    pose.pose.orientation.x = q.x();
                    pose.pose.orientation.y = q.y();
                    pose.pose.orientation.z = q.z();
                    pose.pose.orientation.w = q.w();
                }
                path.poses.push_back(pose);
            }
        }

        Buffer<Elem> viewpoint_dist(const flann::Matrix<Elem>& points)
        {
            Timer t;
            Buffer<Elem> dist(points.rows);
            std::vector<Elem> vp_copy;
            {
                Lock lock(viewpoints_mutex_);
                if (viewpoints_.empty())
                {
                    ROS_WARN("No viewpoints gathered. Return infinity.");
                    std::fill(dist.begin(), dist.end(), std::numeric_limits<Elem>::infinity());
                    return dist;
                }
                vp_copy = viewpoints_;
            }
            size_t n_vp = vp_copy.size() / 3;
            ROS_INFO("Number of viewpoints: %lu.", n_vp);
            flann::Matrix<Elem> vp(vp_copy.data(), n_vp, 3);
//            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeIndexParams(2));
            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeSingleIndexParams());
            vp_index.buildIndex();
            Query<Elem> vp_query(vp_index, points, 1);
            return vp_query.dist_buf_;
        }

        Buffer<Elem> other_viewpoint_dist(const flann::Matrix<Elem>& points)
        {
            Timer t;
            Buffer<Elem> dist(points.rows);
            std::vector<Elem> vp_copy;
            {
                Lock lock(viewpoints_mutex_);
                if (other_viewpoints_.empty())
                {
                    ROS_WARN("No viewpoints gathered from other robots. Return infinity.");
                    std::fill(dist.begin(), dist.end(), std::numeric_limits<Elem>::infinity());
                    return dist;
                }
                vp_copy = other_viewpoints_;
            }
            size_t n_vp = vp_copy.size() / 3;
            ROS_INFO("Number of viewpoints from other robots: %lu.", n_vp);
            flann::Matrix<Elem> vp(vp_copy.data(), n_vp, 3);
//            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeIndexParams(2));
            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeSingleIndexParams());
            vp_index.buildIndex();
            Query<Elem> vp_query(vp_index, points, 1);
            return vp_query.dist_buf_;
        }

        void input_map_received(const sensor_msgs::PointCloud2& cloud)
        {

            Timer t;
            const size_t n_pts = cloud.height * cloud.width;
            sensor_msgs::PointCloud2ConstIterator<Elem> it_points(cloud, "x");
//            const flann::Matrix<Elem> points(const_cast<Elem*>(&it_points[0]), n_pts, 3, cloud.point_step);
            sensor_msgs::PointCloud2ConstIterator<Elem> it_normals(cloud, "normal_x");
//            const flann::Matrix<Elem> normals(const_cast<Elem*>(&it_normals[0]), n_pts, 3, cloud.point_step);
            Buffer<Elem> points_buf(3 * n_pts);
            Buffer<Elem> normals_buf(3 * n_pts);
            auto it_points_dst = points_buf.begin();
            auto it_normals_dst = normals_buf.begin();
            for (size_t i = 0; i < n_pts; ++i, ++it_points, ++it_normals)
            {
                *it_points_dst++ = it_points[0];
                *it_points_dst++ = it_points[1];
                *it_points_dst++ = it_points[2];
                *it_normals_dst++ = it_normals[0];
                *it_normals_dst++ = it_normals[1];
                *it_normals_dst++ = it_normals[2];
            }
            flann::Matrix<Elem> points(points_buf.begin(), n_pts, 3);
            flann::Matrix<Elem> normals(normals_buf.begin(), n_pts, 3);
            ROS_INFO("Copy of %lu points and normals: %.3f s.", n_pts, t.seconds_elapsed());
        }

//        bool plan(const geometry_msgs::PoseStamped& start)
        bool plan(nav_msgs::GetPlanRequest& req, nav_msgs::GetPlanResponse& res)
        {
            Timer t;
            ROS_DEBUG("Planning request received. Plan from [%.2f, %.2f, %.2f] to [%.1f, %.2f, %.2f].",
                    req.start.pose.position.x, req.start.pose.position.y, req.start.pose.position.z,
                    req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z);
            {
                Lock lock(last_request_mutex_);
                last_request_ = req;
            }
            geometry_msgs::PoseStamped start = req.start;
            if (std::isnan(start.pose.position.x)
                    || std::isnan(start.pose.position.y)
                    || std::isnan(start.pose.position.z))
            {
                try
                {
                    auto tf = tf_->lookupTransform(map_frame_, robot_frame_,
                            ros::Time::now(), ros::Duration(5.));
                    transform_to_pose(tf, start);
                }
                catch (const tf2::TransformException& ex)
                {
                    ROS_ERROR("Could not get robot %s position in map %s: %s.",
                            robot_frame_.c_str(), map_frame_.c_str(), ex.what());
                    return false;
                }
            }
            const geometry_msgs::PoseStamped goal = req.goal;

            FlannMat points;
            FlannMat normals;
            FlannMat viewpoints;
            FlannIndexPtr index;
            Buffer<uint16_t> occupied;
            Buffer<uint16_t> empty;
            map_.snapshot(points, normals, viewpoints, index, occupied, empty);

            size_t min_map_points = 64;
            if (points.rows < min_map_points)
            {
                ROS_ERROR("Cannot plan in map with %lu < %lu points.", points.rows, min_map_points);
                return false;
            }

            // Moved to input_map_received above.

            // Initialize debug cloud for visualization of intermediate results.
            sensor_msgs::PointCloud2 debug_cloud;
            create_debug_cloud(points, normals, debug_cloud);
            debug_cloud.header.frame_id = map_frame_;
            // TODO: Refresh on sending?
            debug_cloud.header.stamp = ros::Time::now();
            // Reconstruct original 2D shape.
            debug_cloud.height = 1;
            debug_cloud.width = points.rows;

            // Compute preliminary point labels based on normals.
            Graph g(points, normals, occupied, empty, max_pitch_, max_roll_, uint16_t(empty_ratio_));
            g.k_ = neighborhood_knn_;
            g.radius_ = neighborhood_radius_;
            std::vector<Elem> robots;
            if (filter_robots_)
            {
                // Don't wait for robot positions for planning.
                find_robots(map_frame_, ros::Time(), robots, 0.f);
            }
            g.compute_occupancy_labels(robots);
            // Construct NN graph.
            g.build_index();
            g.compute_graph(neighborhood_knn_, neighborhood_radius_);
//            g.recompute_normals(min_normal_pts_, normal_radius_);
            g.compute_graph_features(min_normal_pts_, normal_radius_, edge_min_centroid_offset_);
            fill_field("num_normal_pts", g.num_normal_pts_.begin(), debug_cloud);
            fill_field("ground_diff_std", g.ground_diff_std_.begin(), debug_cloud);
            g.compute_normal_labels();
            fill_field("normal_label", g.labels_.begin(), debug_cloud);
            normal_label_cloud_pub_.publish(debug_cloud);
            // Adjust points labels using constructed NN graph.
//            g.compute_final_labels(max_nn_height_diff_);
            g.compute_final_labels(max_nn_height_diff_, clearance_low_, clearance_high_, min_points_obstacle_,
                    max_ground_diff_std_, max_ground_abs_diff_mean_, min_dist_to_obstacle_);
            fill_field("ground_diff_min", g.ground_diff_min_.begin(), debug_cloud);
            fill_field("ground_diff_max", g.ground_diff_max_.begin(), debug_cloud);
            fill_field("ground_abs_diff_mean", g.ground_abs_diff_mean_.begin(), debug_cloud);
            fill_field("num_obstacle_pts", g.num_obstacle_pts_.begin(), debug_cloud);
            fill_field("final_label", g.labels_.begin(), debug_cloud);
            final_label_cloud_pub_.publish(debug_cloud);

            // Use the nearest traversable point to robot as the starting point.
            Vec3 start_position(start.pose.position.x, start.pose.position.y, start.pose.position.z);
            size_t tol = 32;
            if (req.tolerance > 0.)
            {
                tol = std::min(size_t(req.tolerance), g.points_.rows);
            }
            Query<Elem> start_query(g.points_index_, flann::Matrix<Elem>(start_position.data(), 1, 3), tol);
            // Get traversable points.
            std::vector<Elem> traversable;
            traversable.reserve(tol);
            for (const auto& v: start_query.nn_buf_)
            {
                if (g.labels_[v] == TRAVERSABLE || g.labels_[v] == EDGE)
                {
                    traversable.push_back(v);
                }
            }
            if (traversable.empty())
            {
                ROS_ERROR("No traversable point near start [%.1f, %.1f, %.1f].",
                        start_position.x(), start_position.y(), start_position.z());
                return false;
            }
//            Vertex v_start = start_query.nn_buf_[0];
            // Get random traversable point as start.
            Vertex v_start = traversable[std::rand() % traversable.size()];
//            Elem min_dist = std::numeric_limits<Elem>::infinity();
//            for (const auto& v: start_query.nn_buf_)
//            {
//                if (g.labels_[v] != TRAVERSABLE)
//                    continue;
//                if (g.labels_[v] )
//                Elem dist = (ConstVec3Map(points[v]) - start_position).norm();
//
//                if (dist < min_dist)
//                {
//                    v_start = v;
//                    min_dist = dist;
//                }
//            }
            ROS_INFO("Planning from [%.1f, %.1f, %.1f], robot at [%.1f, %.1f, %.1f], %lu traversable points nearby.",
                    g.points_[v_start][0], g.points_[v_start][1], g.points_[v_start][2],
                    start_position.x(), start_position.y(), start_position.z(), traversable.size());

            // TODO: Append starting pose as a special vertex with orientation dependent edges.
            // Note, that for some worlds and robots, the neighborhood must be quite large to get traversable points.
            // See e.g. X1 @ cave_circuit_practice_01.

            // Plan in NN graph with approx. travel time costs.
            Buffer<Vertex> predecessor(g.num_vertices());
            Buffer<Elem> path_costs(g.num_vertices());
            EdgeCosts edge_costs(g);
            boost::typed_identity_property_map<Vertex> index_map;

            Timer t_dijkstra;
            // boost::dijkstra_shortest_paths(g, ::Graph::V(0),
            boost::dijkstra_shortest_paths_no_color_map(g, v_start,
//                    &predecessor[0], &path_costs[0], edge_costs,
                    predecessor.begin(), path_costs.begin(), edge_costs,
                    index_map,
                    std::less<Elem>(), boost::closed_plus<Elem>(), std::numeric_limits<Elem>::infinity(), Elem(0.),
                    boost::dijkstra_visitor<boost::null_visitor>());
            ROS_DEBUG("Dijkstra (%u pts): %.3f s.", g.num_vertices(), t_dijkstra.seconds_elapsed());
            fill_field("path_cost", path_costs.begin(), debug_cloud);
            path_cost_cloud_pub_.publish(debug_cloud);

            // If planning for given goal, get the closest feasible path.
            if (std::isfinite(goal.pose.position.x)
                    && std::isfinite(goal.pose.position.y)
                    && std::isfinite(goal.pose.position.z))
            {
                Vec3 goal_position(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
                Vertex v_goal = INVALID_VERTEX;
                Elem best_dist = std::numeric_limits<Elem>::infinity();
                for (Index v = 0; v < path_costs.size(); ++v)
                {
                    if (std::isinf(path_costs[v]))
                    {
                        continue;
                    }
                    Elem dist = (ConstVec3Map(points[v]) - goal_position).norm();
                    if (dist < best_dist)
                    {
                        v_goal = v;
                        best_dist = dist;
                    }
                }
//                Vec3 goal_position(goal.pose.position.x, goal.pose.position.y, goal.pose.position.z);
//                size_t tol = std::min(size_t(req.tolerance), g.points_.rows);
//                Query<Elem> goal_query(g.points_index_, flann::Matrix<Elem>(goal_position.data(), 1, 3), tol);
//                // Get traversable points.
//                std::vector<Elem> traversable;
//                traversable.reserve(tol);
//                for (const auto& v: goal_query.nn_buf_)
//                {
//                    if (g.labels_[v] == TRAVERSABLE || g.labels_[v] == EDGE)
//                    {
//                        traversable.push_back(v);
//                    }
//                }
//                if (traversable.empty())
//                {
//                    ROS_ERROR("No traversable point near goal [%.1f, %.1f, %.1f].",
//                            goal_position.x(), goal_position.y(), goal_position.z());
//                    return false;
//                }
//                // Get random traversable point as goal.
//                v_goal = traversable[std::rand() % traversable.size()];
//
//                ROS_INFO("Planning to [%.1f, %.1f, %.1f], %lu traversable points nearby.",
//                        g.points_[v_goal][0], g.points_[v_goal][1], g.points_[v_goal][2],
//                        goal_position.x(), goal_position.y(), goal_position.z(), traversable.size());

                if (v_goal == INVALID_VERTEX)
                {
                    ROS_WARN("No feasible path to [%.2f, %.2f, %.2f] found (%.3f s).",
                            goal_position.x(), goal_position.y(), goal_position.z(), t.seconds_elapsed());
                    return false;
                }
                std::vector<Vertex> path_indices;
                trace_path_indices(v_start, v_goal, predecessor, path_indices);
                res.plan.header.frame_id = map_frame_;
                res.plan.header.stamp = ros::Time::now();
                res.plan.poses.push_back(start);
                append_path(path_indices, points, normals, res.plan);
                ROS_DEBUG("Path of length %lu (%.3f s).", res.plan.poses.size(), t.seconds_elapsed());
                return true;
            }

            // Compute vertex utility as minimum observation distance.
            Buffer<Elem> vp_dist = viewpoint_dist(points);
            Buffer<Elem> other_vp_dist = other_viewpoint_dist(points);
            assert(vp_dist.size() == points.rows);
            assert(other_vp_dist.size() == points.rows);
            Buffer<Elem> utility(vp_dist.size());

            auto it_vp = vp_dist.begin();
            auto it_other_vp = other_vp_dist.begin();
            auto it_utility = utility.begin();
            for (size_t i = 0; i < utility.size(); ++i, ++it_vp, ++it_other_vp, ++it_utility)
            {
                // Multiply the (clipped) Euclidean distance to encourage exploration.
//                *it = 3.f * std::min(std::max(std::sqrt(*it) - 2.f * neighborhood_radius_, 0.f), 5.f);
//                *it = std::min(std::max(std::sqrt(*it) - min_vp_distance_, 0.f), max_vp_distance_) / max_vp_distance_;
                const auto vp_dist = std::sqrt(*it_vp);
                const auto vp_dist_all = std::min(vp_dist, std::sqrt(*it_other_vp));
                const auto util = std::min(std::max(vp_dist - min_vp_distance_, 0.f), max_vp_distance_) / max_vp_distance_;
                const auto util_all = std::min(std::max(vp_dist_all - min_vp_distance_, 0.f), max_vp_distance_) / max_vp_distance_;
                *it_utility = std::max(self_factor_ * util, util_all);
                // Multiply with edge neighbors.
                *it_utility *= (1 + g.num_edge_neighbors_[i]);
                // Prefer frontiers in a specific subspace (e.g. positive x).
                // TODO: Ensure frame subt is used here.
//                *it_utility /= std::max(-points[i][0] + 9.f, 1.f);
                if (points[i][0] >= -60. && points[i][0] <= 0.
                        && points[i][1] >= -30. && points[i][1] <= 30.
                        && points[i][2] >= -30. && points[i][0] <= 30.)
                {
                    Elem dist_from_origin = ConstVec3Map(points[i]).norm();
                    *it_utility /= (1. + std::pow(dist_from_origin, 4.f));
                }
            }
            fill_field("utility", utility.begin(), debug_cloud);
            utility_cloud_pub_.publish(debug_cloud);

            // Publish final cost cloud.
            Buffer<Elem> final_costs(path_costs.size());
            Elem goal_cost = std::numeric_limits<Elem>::infinity();
            Vertex v_goal = INVALID_VERTEX;
            Vertex v = 0;
            auto it_path_cost = path_costs.begin();
            it_utility = utility.begin();
            auto it_final_cost = final_costs.begin();
            for (; it_path_cost != path_costs.end(); ++v, ++it_path_cost, ++it_utility, ++it_final_cost)
            {
                // *it_final_cost = *it_path_cost - *it_utility;
                *it_final_cost = std::log(*it_path_cost / (*it_utility + 1e-6f));

                // Avoid short paths and paths with no utility.
                if (*it_final_cost < goal_cost && *it_path_cost > neighborhood_radius_ && *it_utility > 0.)
                {
                    goal_cost = *it_final_cost;
                    v_goal = v;
                }
            }
            if (v_goal == INVALID_VERTEX)
            {
                ROS_ERROR("No valid path/goal found.");
                return false;
            }

            ROS_INFO("Goal position: %.1f, %.1f, %.1f m: cost %.3g, utility %.3g, final cost %.3g.",
                    points[v_goal][0], points[v_goal][1], points[v_goal][2],
                    path_costs[v_goal], utility[v_goal], final_costs[v_goal]);
            fill_field("final_cost", final_costs.begin(), debug_cloud);
            final_cost_cloud_pub_.publish(debug_cloud);

            // TODO: Remove inf from path cost for visualization?

            std::vector<Vertex> path_indices;
            trace_path_indices(v_start, v_goal, predecessor, path_indices);

//            nav_msgs::Path path;
            res.plan.header.frame_id = map_frame_;
            res.plan.header.stamp = ros::Time::now();
            res.plan.poses.push_back(start);
            append_path(path_indices, points, normals, res.plan);
            ROS_DEBUG("Path of length %lu (%.3f s).", res.plan.poses.size(), t.seconds_elapsed());
            return true;
        }

        void cloud_received(const sensor_msgs::PointCloud2::ConstPtr& cloud)
        {
            ROS_INFO("Cloud received (%u points).", cloud->height * cloud->width);

            // TODO: Build map from all aligned input clouds (interp tf).
            // TODO: Recompute normals.
            if (cloud->row_step != cloud->point_step * cloud->width)
            {
                ROS_ERROR("Skipping cloud with unsupported row step.");
                return;
            }
            const auto age = (ros::Time::now() - cloud->header.stamp).toSec();
            if (age > max_cloud_age_)
            {
                ROS_INFO("Skipping cloud %.1f s > %.1f s old.", age, max_cloud_age_);
                return;
            }
            if (!map_frame_.empty() && map_frame_ != cloud->header.frame_id)
            {
                ROS_ERROR("Cloud frame %s does not match specified map frame %s.",
                        cloud->header.frame_id.c_str(), map_frame_.c_str());
                return;
            }

            // TODO: Allow x[3] or x,y,z and normal[3] or normal_x,y,z.
            const auto field_x = find_field(*cloud, position_name_);
            if (!field_x)
            {
                ROS_ERROR("Skipping cloud without positions.");
                return;
            }
            if (field_x->datatype != sensor_msgs::PointField::FLOAT32)
            {
                ROS_ERROR("Skipping cloud with unsupported type %u.", field_x->datatype);
                return;
            }

            const auto field_nx = find_field(*cloud, normal_name_);
            if (!field_nx)
            {
                ROS_ERROR("Skipping cloud without normals.");
                return;
            }
            if (field_nx->datatype != sensor_msgs::PointField::FLOAT32)
            {
                ROS_ERROR("Skipping cloud with unsupported normal type %u.", field_nx->datatype);
                return;
            }

            geometry_msgs::PoseStamped start;
            try
            {
                auto tf = tf_->lookupTransform(cloud->header.frame_id, robot_frame_, ros::Time::now(),
                        ros::Duration(5.));
                transform_to_pose(tf, start);
            }
            catch (const tf2::TransformException& ex)
            {
                ROS_ERROR("Could not get robot position: %s.", ex.what());
                return;
            }
            // TODO: Update whole map with the input map cloud.
//            plan(*cloud, start);
        }

        void planning_timer_cb(const ros::TimerEvent& event)
        {
            ROS_DEBUG("Planning timer callback.");
            Timer t;
            nav_msgs::GetPlanRequest req;
            {
                Lock lock(last_request_mutex_);
                req = last_request_;
            }
            nav_msgs::GetPlanResponse res;
            if (!plan(req, res))
            {
                return;
            }
            path_pub_.publish(res.plan);
            ROS_INFO("Planning robot %s path (%lu poses) in map %s: %.3f s.",
                    robot_frame_.c_str(), res.plan.poses.size(), map_frame_.c_str(), t.seconds_elapsed());
        }

        void find_robots(const std::string& frame, const ros::Time& stamp, std::vector<Elem>& robots, float timeout)
        {
            robots.reserve(robot_frames_.size());
            for (const auto kv: robot_frames_)
            {
                if (robot_frame_ == kv.second)
                {
                    continue;
                }
                ros::Duration timeout_duration(std::max(timeout - (ros::Time::now() - stamp).toSec(), 0.));
                geometry_msgs::TransformStamped tf;
                try
                {
                    tf = tf_->lookupTransform(frame, stamp, kv.second, stamp, map_frame_, timeout_duration);
                }
                catch (const tf2::TransformException& ex)
                {
                    ROS_WARN("Could not get %s pose in %s: %s.",
                            kv.second.c_str(), frame.c_str(), ex.what());
                    continue;
                }
                robots.push_back(tf.transform.translation.x);
                robots.push_back(tf.transform.translation.y);
                robots.push_back(tf.transform.translation.z);
                ROS_INFO("Robot %s found in %s at [%.1f, %.1f, %.1f].", kv.second.c_str(), frame.c_str(),
                        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
            }
        }

        void input_cloud_received(const sensor_msgs::PointCloud2::ConstPtr& cloud)
        {
            const Index n_pts = cloud->height * cloud->width;
            ROS_DEBUG("Input cloud from %s with %u points received.",
                    cloud->header.frame_id.c_str(), n_pts);
            double timeout_ = 5.;
            ros::Duration timeout(std::max(timeout_ - (ros::Time::now() - cloud->header.stamp).toSec(), 0.));
            geometry_msgs::TransformStamped tf;
            try
            {
                tf = tf_->lookupTransform(map_frame_, cloud->header.frame_id, cloud->header.stamp, timeout);
            }
            catch (const tf2::TransformException& ex)
            {
                ROS_ERROR("Could not transform input cloud from %s into map %s: %s.",
                        cloud->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
                return;
            }
            Timer t;
            Quat rotation(tf.transform.rotation.w,
                    tf.transform.rotation.x,
                    tf.transform.rotation.y,
                    tf.transform.rotation.z);
            Eigen::Translation3f translation(tf.transform.translation.x,
                    tf.transform.translation.y,
                    tf.transform.translation.z);
            Eigen::Affine3f transform = translation * rotation;

//            Buffer<Elem> robots;
            std::vector<Elem> robots;
            if (filter_robots_)
            {
                find_robots(cloud->header.frame_id, cloud->header.stamp, robots, 3.f);
            }

            Vec3 origin = transform * Vec3(0., 0., 0.);
            flann::Matrix<Elem> origin_mat(origin.data(), 1, 3);

            sensor_msgs::PointCloud2ConstIterator<Elem> it_points(*cloud, position_name_);
            Buffer<Elem> points_buf(3 * n_pts);
            auto it_points_dst = points_buf.begin();
            Index n_added = 0;
            for (Index i = 0; i < n_pts; ++i, ++it_points)
            {
                ConstVec3Map src(&it_points[0]);
                if (src.norm() < 1.f || src.norm() > 25.f)
                {
                    continue;
                }
                // Filter robots.
                bool remove = false;
                for (Index i = 0; i + 2 < robots.size(); i += 3)
                {
                    if ((src - ConstVec3Map(&robots[i])).norm() < 1.f)
                    {
                        remove = true;
                        break;
                    }
                }
                if (remove)
                {
                    continue;
                }
                Vec3Map dst(it_points_dst);
                dst = transform * src;
                it_points_dst += 3;
                ++n_added;
            }
            Index min_pts = 16;
            if (n_added < min_pts)
            {
                ROS_INFO("Discarding input cloud: not enough points to merge: %u < %u.", n_added, min_pts);
                return;
            }
            flann::Matrix<Elem> points(points_buf.begin(), n_added, 3);
            map_.merge(points, origin_mat);
            ROS_DEBUG("Input cloud with %u points merged: %.3f s.", n_added, t.seconds_elapsed());

            sensor_msgs::PointCloud2 map_cloud;
            map_cloud.header.frame_id = map_frame_;
            map_cloud.header.stamp = cloud->header.stamp;
            map_.create_cloud(map_cloud);
            map_pub_.publish(map_cloud);
        }

    protected:
        typedef std::recursive_mutex Mutex;
        typedef std::lock_guard<Mutex> Lock;

        ros::NodeHandle& nh_;
        ros::NodeHandle& pnh_;
        std::unique_ptr<tf2_ros::Buffer> tf_;
        std::unique_ptr<tf2_ros::TransformListener> tf_sub_;
        ros::Publisher normal_label_cloud_pub_;
        ros::Publisher final_label_cloud_pub_;
        ros::Publisher path_cost_cloud_pub_;
        ros::Publisher utility_cloud_pub_;
        ros::Publisher final_cost_cloud_pub_;
        ros::Publisher path_pub_;
        ros::Publisher minpos_path_pub_;
        ros::Publisher viewpoints_pub_;
        ros::Publisher other_viewpoints_pub_;
        ros::Subscriber cloud_sub_;

        std::vector<ros::Subscriber> input_cloud_subs_;
        ros::Publisher map_pub_;
        ros::Timer planning_timer_;
        ros::ServiceServer get_plan_service_;
        Mutex last_request_mutex_;
        nav_msgs::GetPlanRequest last_request_;
        ros::Timer viewpoints_update_timer_;
        ros::WallTimer update_params_timer_;

        std::string position_name_;
        std::string normal_name_;

        std::string map_frame_;
        std::string robot_frame_;
        std::map<std::string, std::string> robot_frames_;
        float max_cloud_age_;
        float max_pitch_;
        float max_roll_;
        int empty_ratio_;
        bool filter_robots_;

        int neighborhood_knn_;
        float neighborhood_radius_;
        int min_normal_pts_;
        float normal_radius_;

        float max_nn_height_diff_;
        float clearance_low_;
        float clearance_high_;
        float min_points_obstacle_;
        float max_ground_diff_std_;
        float max_ground_abs_diff_mean_;
        float edge_min_centroid_offset_;
        float min_dist_to_obstacle_;

        Mutex viewpoints_mutex_;
        float viewpoints_update_freq_;
        std::vector<Elem> viewpoints_;  // 3xN
        std::vector<Elem> other_viewpoints_;  // 3xN
//        std::map<std::string, Vec3> last_positions_;
        float min_vp_distance_;
        float max_vp_distance_;
        float self_factor_;
        float planning_freq_;

        int queue_size_;
        Mutex map_mutex_;
        Map map_;
    };

}  // namespace naex

#endif //NAEX_PLANNER_H
