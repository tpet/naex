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
#include <nav_msgs/Path.h>
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
    // Vertex and edge indices
    typedef uint32_t Vertex;
    typedef uint32_t Edge;
    // Edge cost or length
    typedef Elem Cost;

    enum Label
    {
        TRAVERSABLE,
        UNKNOWN,
        OBSTACLE
    };
    typedef Buffer<uint8_t> Labels;

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
        modifier.setPointCloud2Fields(14,
                "x", 1, sensor_msgs::PointField::FLOAT32,
                "y", 1, sensor_msgs::PointField::FLOAT32,
                "z", 1, sensor_msgs::PointField::FLOAT32,
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
                "final_cost", 1, sensor_msgs::PointField::FLOAT32);
        modifier.resize(points.rows);

        sensor_msgs::PointCloud2Iterator<float>
                x_it(cloud, "x"),
                y_it(cloud, "y"),
                z_it(cloud, "z");
        for (size_t i = 0; i < points.rows; ++i, ++x_it, ++y_it, ++z_it)
        {
            *x_it = points[i][0];
            *y_it = points[i][1];
            *z_it = points[i][2];
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
        Query(const flann::Index<flann::L2_3D<T>>& index, const flann::Matrix<T>& queries, int k = 1):
                nn_buf_(queries.rows * k),
                dist_buf_(queries.rows * k),
                nn_(nn_buf_.begin(), queries.rows, k),
                dist_(dist_buf_.begin(), queries.rows, k)
        {
            flann::SearchParams params;
            params.cores = 0;
            index.knnSearch(queries, nn_, dist_, k, params);
        }
        Buffer<int> nn_buf_;
        Buffer<T> dist_buf_;
        flann::Matrix<int> nn_;
        flann::Matrix<T> dist_;
    };
    template<typename T>
    Query<T> query(const flann::Index<flann::L2_3D<T>>& index, const flann::Matrix<T>& queries, int k = 1)
    {
        return Query<T>(index, queries, k);
    }

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
                float max_pitch, float max_roll):
                points_(points),
                points_index_(points_, flann::KDTreeIndexParams(2)),
                normals_(normals),
                labels_(points_.rows),
                max_pitch_(max_pitch),
                max_roll_(max_roll)
        {
            // TODO: First, split points to (maybe) traversable and (hard) obstacles.
            // Traversable:
            // - approx. horizontal based on normal (with correct orientation).
            // - nearby points below ground
            // Obstacles:
            // - approx. vertical based on normal (allow opposite orientation).
//            compute_normal_labels();
        }

        /** Traversability based on normal direction. */
        void compute_normal_labels()
        {
            Timer t;
            // Maximum slope allowed in some direction.
            auto max_slope = std::max(max_pitch_, max_roll_);
            auto min_z = std::cos(max_slope);
            size_t n_traverable = 0, n_obstacle = 0, n_unknown = 0;
            for (size_t i = 0; i < normals_.rows; ++i)
            {
                if (std::abs(normals_[i][2]) >= min_z)
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
            ROS_INFO("Normal labels (%lu pts): %lu trav., %lu obs., %lu unk. (%.3f s).",
                    normals_.rows, n_traverable, n_obstacle, n_unknown, t.seconds_elapsed());
        }

        void compute_graph_features(size_t min_normal_pts, Elem radius)
        {
            Timer t;
            num_normal_pts_.resize(nn_.rows);
//            ground_diff_min_.resize(nn_.rows);
//            ground_diff_max_.resize(nn_.rows);
            ground_diff_std_.resize(nn_.rows);

            size_t n_computed = 0;
            for (Vertex v0 = 0; v0 < nn_.rows; ++v0)
            {
                Vec3 mean = Vec3::Zero();
//                size_t n = 0;
                num_normal_pts_[v0] = 0;
                for (size_t j = 0; j < nn_.cols; ++j)
                {
                    if (dist_[v0][j] <= radius)
                    {
                        mean += Vec3Map(points_[nn_[v0][j]]);
//                        ++n;
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
                    if (dist_[v0][j] <= radius)
                    {
                        auto pc = (Vec3Map(points_[nn_[v0][j]]) - mean);
                        cov += pc * pc.transpose();
                    }
                }
//                cov /= (n + 1);
                cov /= (num_normal_pts_[v0] + 1);
                Eigen::SelfAdjointEigenSolver<Mat3> solver(cov);
                // solver.eigenvalues();
                Vec3Map normal(normals_[v0]);
                normal = solver.eigenvectors().col(0);
                ground_diff_std_[v0] = std::sqrt(solver.eigenvalues()(0));
//                ground_diff_min_
//                ++n_computed;
            }
            ROS_INFO("Normals recomputed for %lu points from %lu nn within %.2g m: %.3f s.",
                    nn_.rows, nn_.cols, radius_, t.seconds_elapsed());
        }

        /** Traversability based on NN graph. */
        void compute_final_labels(Elem max_nn_height_diff_, Elem clearance_low, Elem clearance_high,
                Vertex min_points_obstacle, Elem max_ground_diff_std, Elem max_ground_abs_diff_mean)
        {
            Timer t;
            ground_diff_min_.resize(nn_.rows);
            ground_diff_max_.resize(nn_.rows);
            ground_abs_diff_mean_.resize(nn_.rows);
            num_obstacle_pts_.resize(nn_.rows);
            // Maximum slope allowed in some direction.
            size_t n_traverable = 0, n_unknown = 0, n_obstacle = 0;
//            ROS_INFO("NN rows: %lu, cols %lu", nn_.rows, nn_.cols);
            for (Vertex v0 = 0; v0 < nn_.rows; ++v0)
            {
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
                    // Avoid driving near obstacles.
                    const auto v1 = nn_[v0][j];
                    if (labels_[v0] == TRAVERSABLE && labels_[v1] == OBSTACLE && dist_[v0][j] <= radius_)
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
            ROS_INFO("Final graph-adjusted labels (%lu pts): %lu traversable, %lu unknown, %lu obstacle (%.3f s).",
                    normals_.rows, n_traverable, n_unknown, n_obstacle, t.seconds_elapsed());
        }

        void build_index()
        {
            Timer t;
            points_index_.buildIndex();
            ROS_INFO("Building index for %lu pts: %.3f s.", points_.rows, t.seconds_elapsed());
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
            ROS_INFO("NN graph (%lu pts): %.3f s.", points_.rows, t.seconds_elapsed());
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
        Buffer<uint8_t> num_normal_pts_;
        // from ball neighborhood
        Buffer<Elem> ground_diff_std_;
        // circle in ground plane
        Buffer<Elem> ground_diff_min_;
        Buffer<Elem> ground_diff_max_;
        Buffer<Elem> ground_abs_diff_mean_;
        Buffer<uint8_t> num_obstacle_pts_;
        Buffer<uint8_t> labels_;

        // NN and distances
        int k_;
        Elem radius_;
        Buffer<int> nn_buf_;
        Buffer<Elem> dist_buf_;
        flann::Matrix<int> nn_;
        flann::Matrix<Elem> dist_;

        float max_pitch_;
        float max_roll_;
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
                other_viewpoints_(),
                min_vp_distance_(1.5),
                max_vp_distance_(5.),
                self_factor_(0.25),
                queue_size_(5.)
        {
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

            pnh_.param("viewpoints_update_freq", viewpoints_update_freq_, viewpoints_update_freq_);
            pnh_.param("min_vp_distance", min_vp_distance_, min_vp_distance_);
            pnh_.param("max_vp_distance", max_vp_distance_, max_vp_distance_);
            pnh_.param("self_factor", self_factor_, self_factor_);

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
            tf_.reset(new tf2_ros::Buffer(ros::Duration(10.)));
            tf_sub_.reset(new tf2_ros::TransformListener(*tf_));
            viewpoints_update_timer_ =  nh_.createTimer(ros::Rate(viewpoints_update_freq_),
                    &Planner::gather_viewpoints, this);
            update_params_timer_ = nh_.createWallTimer(ros::WallDuration(2.0),
                    &Planner::update_params, this);

            normal_label_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("normal_label_cloud", 5);
            final_label_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("final_label_cloud", 5);
            path_cost_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("path_cost_cloud", 5);
            utility_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("utility_cloud", 5);
            final_cost_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("final_cost_cloud", 5);

            path_pub_ = nh_.advertise<nav_msgs::Path>("path", 5);
            minpos_path_pub_ = nh_.advertise<nav_msgs::Path>("minpos_path", 5);

            viewpoints_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("viewpoints", 5);
            other_viewpoints_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("other_viewpoints", 5);
            cloud_sub_ = nh_.subscribe("cloud", queue_size_, &Planner::cloud_received, this);
        }

        void gather_viewpoints(const ros::TimerEvent& event)
        {
            ROS_INFO("Gathering viewpoints.");
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
            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeIndexParams(2));
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
            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeIndexParams(2));
            vp_index.buildIndex();
            Query<Elem> vp_query(vp_index, points, 1);
            return vp_query.dist_buf_;
        }

        void plan(const sensor_msgs::PointCloud2& cloud, const geometry_msgs::PoseStamped& start)
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

            // Initialize debug cloud for visualization of intermediate results.
            sensor_msgs::PointCloud2 debug_cloud;
            create_debug_cloud(points, normals, debug_cloud);
            debug_cloud.header = cloud.header;
            // Reconstruct original 2D shape.
            debug_cloud.height = cloud.height;
            debug_cloud.width = cloud.width;

            // Compute preliminary point labels based on normals.
            Graph g(points, normals, max_pitch_, max_roll_);
            // Construct NN graph.
            g.build_index();
            g.compute_graph(neighborhood_knn_, neighborhood_radius_);
//            g.recompute_normals(min_normal_pts_, normal_radius_);
            g.compute_graph_features(min_normal_pts_, normal_radius_);
            fill_field("num_normal_pts", g.num_normal_pts_.begin(), debug_cloud);
            fill_field("ground_diff_std", g.ground_diff_std_.begin(), debug_cloud);
            g.compute_normal_labels();
            fill_field("normal_label", g.labels_.begin(), debug_cloud);
            normal_label_cloud_pub_.publish(debug_cloud);

            // Adjust points labels using constructed NN graph.
//            g.compute_final_labels(max_nn_height_diff_);
            g.compute_final_labels(max_nn_height_diff_, clearance_low_, clearance_high_, min_points_obstacle_,
                    max_ground_diff_std_, max_ground_abs_diff_mean_);
            fill_field("ground_diff_min", g.ground_diff_min_.begin(), debug_cloud);
            fill_field("ground_diff_max", g.ground_diff_max_.begin(), debug_cloud);
            fill_field("ground_abs_diff_mean", g.ground_abs_diff_mean_.begin(), debug_cloud);
            fill_field("num_obstacle_pts", g.num_obstacle_pts_.begin(), debug_cloud);
            fill_field("final_label", g.labels_.begin(), debug_cloud);
            final_label_cloud_pub_.publish(debug_cloud);

            // Use the nearest traversable point to robot as the starting point.
            Vec3 start_position(start.pose.position.x, start.pose.position.y, start.pose.position.z);
            Query<Elem> start_query(g.points_index_, flann::Matrix<Elem>(start_position.data(), 1, 3), 128);
            Vertex v_start = start_query.nn_buf_[0];
            Elem min_dist = std::numeric_limits<Elem>::infinity();
            for (const auto& v: start_query.nn_buf_)
            {
                if (g.labels_[v] != TRAVERSABLE)
                    continue;
                Elem dist = (ConstVec3Map(points[v]) - start_position).norm();
                if (dist < min_dist)
                {
                    v_start = v;
                    min_dist = dist;
                }
            }
            // TODO: Append starting pose as a special vertex with orientation dependent edges.
            // Note, that for some worlds and robots, the neighborhood must be quite large to get traversable points.
            // See e.g. X1 @ cave_circuit_practice_01.

            // Plan in NN graph with approx. travel time costs.
            Buffer<Vertex> predecessor(g.num_vertices());
            Buffer<Elem> path_costs(g.num_vertices());
            EdgeCosts edge_costs(g);
            boost::typed_identity_property_map<Vertex> index_map;

            t.reset();
//            std::cout << "DIJKSTRA" << std::endl;
            // boost::dijkstra_shortest_paths(g, ::Graph::V(0),
            boost::dijkstra_shortest_paths_no_color_map(g, v_start,
//                    &predecessor[0], &path_costs[0], edge_costs,
                    predecessor.begin(), path_costs.begin(), edge_costs,
                    index_map,
                    std::less<Elem>(), boost::closed_plus<Elem>(), std::numeric_limits<Elem>::infinity(), Elem(0.),
                    boost::dijkstra_visitor<boost::null_visitor>());
            ROS_INFO("Dijkstra (%u pts): %.3f s.", g.num_vertices(), t.seconds_elapsed());
            fill_field("path_cost", path_costs.begin(), debug_cloud);
            path_cost_cloud_pub_.publish(debug_cloud);
//            return;

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
                // Prefer frontiers in a specific subspace (e.g. positive x).
                // TODO: Ensure frame subt is used here.
                *it_utility /= std::max(-points[i][0] + 9.f, 1.f);
            }
            fill_field("utility", utility.begin(), debug_cloud);
            utility_cloud_pub_.publish(debug_cloud);

            // Publish final cost cloud.
            Buffer<Elem> final_costs(path_costs.size());
            Elem goal_cost = std::numeric_limits<Elem>::infinity();
            Vertex v_goal = v_start;
            Vertex v = 0;
            auto it_path_cost = path_costs.begin();
            it_utility = utility.begin();
            auto it_final_cost = final_costs.begin();
            for (; it_path_cost != path_costs.end(); ++v, ++it_path_cost, ++it_utility, ++it_final_cost)
            {
                // *it_final_cost = *it_path_cost - *it_utility;
                *it_final_cost = std::log(*it_path_cost / (*it_utility + 1e-6f));
                // Avoid single pose paths and paths with no utility.
                if (*it_final_cost < goal_cost && *it_path_cost > 0. && *it_utility > 0.)
                {
                    goal_cost = *it_final_cost;
                    v_goal = v;
                }
            }
            ROS_INFO("Goal position: %.1f, %.1f, %.1f m: cost %.3g, utility %.3g, final cost %.3g.",
                    points[v_goal][0], points[v_goal][1], points[v_goal][2],
                    path_costs[v_goal], utility[v_goal], final_costs[v_goal]);
            fill_field("final_cost", final_costs.begin(), debug_cloud);
            final_cost_cloud_pub_.publish(debug_cloud);

            // TODO: Remove inf from path cost for visualization?

            std::vector<Vertex> path_indices;
            trace_path_indices(v_start, v_goal, predecessor, path_indices);

            nav_msgs::Path path;
            path.header.frame_id = cloud.header.frame_id;
            path.header.stamp = ros::Time::now();
            path.poses.push_back(start);
            append_path(path_indices, points, normals, path);
            path_pub_.publish(path);
            ROS_INFO("Path length: %lu.", path.poses.size());
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

            plan(*cloud, start);
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

        std::string position_name_;
        std::string normal_name_;

        std::string map_frame_;
        std::string robot_frame_;
        std::map<std::string, std::string> robot_frames_;
        float max_cloud_age_;
        float max_pitch_;
        float max_roll_;

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

        std::recursive_mutex viewpoints_mutex_;
        float viewpoints_update_freq_;
        ros::Timer viewpoints_update_timer_;
        ros::WallTimer update_params_timer_;
        std::vector<Elem> viewpoints_;  // 3xN
        std::vector<Elem> other_viewpoints_;  // 3xN
//        std::map<std::string, Vec3> last_positions_;
        float min_vp_distance_;
        float max_vp_distance_;
        float self_factor_;

        int queue_size_;
    };

}  // namespace naex

#endif //NAEX_PLANNER_H
