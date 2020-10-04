//
// Created by petrito1 on 10/1/20.
//

#ifndef NAEX_PLANNER_H
#define NAEX_PLANNER_H

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
    typedef naex::Buffer<uint8_t> Labels;

    void transform_to_pose(const geometry_msgs::TransformStamped& tf, geometry_msgs::PoseStamped& pose)
    {
        pose.header = tf.header;
        pose.pose.position.x = tf.transform.translation.x;
        pose.pose.position.y = tf.transform.translation.y;
        pose.pose.position.z = tf.transform.translation.z;
        pose.pose.orientation = tf.transform.rotation;
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

    void create_debug_cloud(
            const flann::Matrix<Elem>& points,
            const flann::Matrix<Elem>& normals,
            const Labels& labels,
            sensor_msgs::PointCloud2& cloud)
    {
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::PointField::FLOAT32,
                                         "y", 1, sensor_msgs::PointField::FLOAT32,
                                         "z", 1, sensor_msgs::PointField::FLOAT32,
                                         "label", 1, sensor_msgs::PointField::UINT8);
        modifier.resize(points.rows);
        sensor_msgs::PointCloud2Iterator<float> x_iter(cloud, "x"), y_iter(cloud, "y"), z_iter(cloud, "z");
        sensor_msgs::PointCloud2Iterator<uint8_t> label_iter(cloud, "label");
        for (size_t i = 0; i < points.rows; ++i, ++x_iter, ++y_iter, ++z_iter, ++label_iter)
        {
            *x_iter = points[i][0];
            *y_iter = points[i][1];
            *z_iter = points[i][2];
            *label_iter = labels[i];
        }
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
                points_index_(points_, flann::KDTreeIndexParams(4)),
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
//                auto slope = std::acos(normals_[i][2]);
//                if (slope <= max_slope)
                if (normals_[i][2] >= min_z)
                {
                    // Approx. horizontal based on normal (with correct orientation).
                    labels_[i] = TRAVERSABLE;
                    ++n_traverable;
                }
//                else if (M_PI - slope > max_slope)
                else if (std::abs(normals_[i][2]) < min_z)
                {
                    // Approx. vertical based on normal (allow orientation mismatch).
                    labels_[i] = OBSTACLE;
                    ++n_obstacle;
                }
                else
                {
                    labels_[i] = UNKNOWN;
                    ++n_unknown;
                }
            }
            ROS_INFO("Normal labels (%lu pts): %lu trav., %lu obs., %lu unk. (%.3f s).",
                    normals_.rows, n_traverable, n_obstacle, n_unknown, t.seconds_elapsed());
        }

        void compute_graph(Vertex k, Elem radius)
        {
            Timer t;
            // TODO: Use point copy prior building index?
            points_index_.buildIndex();
            ROS_INFO("NN index (%lu pts): %.3f s.", points_.rows, t.seconds_elapsed());
            return;
            nn_buf_.resize(num_vertices() * k);
            dist_buf_.resize(num_vertices() * k);
            nn_ = flann::Matrix<int>(nn_buf_.data(), num_vertices(), k);
            dist_ = flann::Matrix<Cost>(dist_buf_.data(), num_vertices(), k);
//            nn_buf_.reset(new int[num_vertices() * k]);
//            dist_buf_.reset(new Elem[num_vertices() * k]);
//            dist_ = flann::Matrix<Cost>(dist_buf_.get(), num_vertices(), k);
//            nn_ = flann::Matrix<int>(nn_buf_.get(), num_vertices(), k);
//            nn_buf_.resize(num_vertices() * k);
//            dist_buf_.resize(num_vertices() * k);
//            dist_ = flann::Matrix<Cost>(dist_buf_.begin(), num_vertices(), k);
//            nn_ = flann::Matrix<int>(nn_buf_.begin(), num_vertices(), k);
//            flann::Matrix<Cost> dist_(dist_buf_.begin(), num_vertices(), k);
//            flann::Matrix<int> nn_(nn_buf_.begin(), num_vertices(), k);
            std::cout << "Buffers initialized." << std::endl;
            t.reset();
            flann::SearchParams params;
//            params.checks = 64;
//            params.max_neighbors = k;
//            points_index_.radiusSearch(points_, nn_, dist_, radius, params);
            points_index_.knnSearch(points_, nn_, dist_, k, params);
            ROS_INFO("NN graph (%lu pts): %.3f s.", points_.rows, t.seconds_elapsed());
        }

        inline Vertex num_vertices() const
        {
            return nn_.rows;
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
        inline Vertex target(const Edge& e) const
        {
            return nn_[source(e)][e % num_edges()];
        }
        inline Cost cost(const Edge& e) const
        {
            const auto v0 = source(e);
            const auto v1 = target(e);
            if (labels_[v0] != TRAVERSABLE || labels_[v1] != TRAVERSABLE)
            {
                return std::numeric_limits<Cost>::infinity();
            }
            Vec2 xy_dir = Vec2Map(points_[v1]) - Vec2Map(points_[v0]);
            Elem height_diff = points_[v1][2] - points_[v0][2];
            Elem inclination = std::atan(height_diff / xy_dir.norm());
            if (inclination > max_pitch_)
            {
                return std::numeric_limits<Cost>::infinity();
            }
            // TODO: Check pitch and roll separately.
            Elem pitch0 = std::acos(normals_[v0][2]);
            if (pitch0 > max_pitch_)
            {
                return std::numeric_limits<Cost>::infinity();
            }
            Elem pitch1 = std::acos(normals_[v0][2]);
            if (pitch1 > max_pitch_)
            {
                return std::numeric_limits<Cost>::infinity();
            }
            float roll0 = 0.f;
            float roll1 = 0.f;
            // Initialize with distance computed in NN search.
            Cost d = dist_[v0][e % num_edges()];
            // Multiple with relative pitch and roll.
            d *= (1.f + (pitch0 + pitch1 + inclination) / 3.f / max_pitch_ + (roll0 + roll1) / 2.f / max_roll_);
            std::cout << v0 << " -> " << v1 << ": " << d << std::endl;
            return d;
        }

        flann::Matrix<Elem> points_;
        flann::Index<flann::L2<Cost>> points_index_;
        flann::Matrix<Elem> normals_;
        naex::Buffer<uint8_t> labels_;

        // NN and distances
//        std::unique_ptr<int[]> nn_buf_;
//        std::unique_ptr<Cost[]> dist_buf_;
//        naex::Buffer<int> nn_buf_;
//        naex::Buffer<Cost> dist_buf_;
        std::vector<int> nn_buf_;
        std::vector<Cost> dist_buf_;
        flann::Matrix<int> nn_;
        flann::Matrix<Cost> dist_;

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
        Planner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
                :
                nh_(nh),
                pnh_(pnh),
                tf_(),
                position_name_("x"),
                normal_name_("normal_x"),
                map_frame_(""),
                robot_frame_("base_footprint"),
                max_cloud_age_(5.),
                max_pitch_(30. / 180. * M_PI),
                max_roll_(30. / 180. * M_PI),
                neighborhood_knn_(12),
                neighborhood_radius_(.5),

                queue_size_(5.)
        {
            configure();
        }

        void configure()
        {
            pnh_.param("position_name", position_name_, position_name_);
            pnh_.param("normal_name", normal_name_, normal_name_);
            pnh_.param("map_frame", map_frame_, map_frame_);
            pnh_.param("robot_frame", robot_frame_, robot_frame_);
            pnh_.param("max_cloud_age", max_cloud_age_, max_cloud_age_);
            pnh_.param("max_pitch", max_pitch_, max_pitch_);
            pnh_.param("max_roll", max_roll_, max_roll_);
            pnh_.param("neighborhood_knn", neighborhood_knn_, neighborhood_knn_);
            pnh_.param("neighborhood_radius", neighborhood_radius_, neighborhood_radius_);

            tf_.reset(new tf2_ros::Buffer(ros::Duration(15.)));
            tf_sub_.reset(new tf2_ros::TransformListener(*tf_));
            normal_labels_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("normal_labels", 5);
            path_pub_ = nh_.advertise<nav_msgs::Path>("path", 5);
            minpos_path_pub_ = nh_.advertise<nav_msgs::Path>("minpos_path", 5);
            cloud_sub_ = nh_.subscribe("cloud", queue_size_, &Planner::cloud_received, this);
        }

        void plan(const sensor_msgs::PointCloud2& cloud, const geometry_msgs::PoseStamped& start)
        {
            ROS_INFO("plan");
            const auto n_pts = cloud.height * cloud.width;
            sensor_msgs::PointCloud2ConstIterator<Elem> it_points(cloud, "x");
            const flann::Matrix<Elem> points(const_cast<Elem*>(&it_points[0]), n_pts, 3, cloud.point_step);

            sensor_msgs::PointCloud2ConstIterator<Elem> it_normals(cloud, "normal_x");
            const flann::Matrix<Elem> normals(const_cast<Elem*>(&it_normals[0]), n_pts, 3, cloud.point_step);

            // Construct NN graph.
            Timer t;
            Graph g(points, normals, max_pitch_, max_roll_);
            ROS_INFO("Creating NN index: %.3f s.", t.seconds_elapsed());
            g.compute_normal_labels();
            sensor_msgs::PointCloud2 normal_labels;
            create_debug_cloud(g.points_, g.normals_, g.labels_, normal_labels);
            normal_labels.header = cloud.header;
            normal_labels.height = cloud.height;
            normal_labels.width = cloud.width;
            normal_labels_pub_.publish(normal_labels);
//            return;

            g.compute_graph(neighborhood_knn_, neighborhood_radius_);
            return;

            // Plan in NN graph with approx. travel time costs.
            std::vector<Vertex> predecessor(boost::num_vertices(g));
            std::vector<Cost> path_costs(boost::num_vertices(g));
            EdgeCosts edge_costs(g);
            boost::typed_identity_property_map<Vertex> index_map;

            std::cout << "DIJKSTRA" << std::endl;
            // boost::dijkstra_shortest_paths(g, ::Graph::V(0),
            boost::dijkstra_shortest_paths_no_color_map(g, Vertex(0),
                    &predecessor[0], &path_costs[0], edge_costs,
                    index_map,
                    std::less<Cost>(), boost::closed_plus<Cost>(), std::numeric_limits<Cost>::infinity(), Cost(0.),
                    boost::dijkstra_visitor<boost::null_visitor>());

            // TODO: Compute vertex utility as minimum observation distance.

            // TODO: Add penalty for initial rotation: + abs(angle_err) / angular_max.

            // TODO: Subtract utility from visiting the frontiers: - observation distance.

            // TODO: Prefer frontiers in a specific subspace.
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
        ros::NodeHandle& nh_;
        ros::NodeHandle& pnh_;
        std::unique_ptr<tf2_ros::Buffer> tf_;
        std::unique_ptr<tf2_ros::TransformListener> tf_sub_;
        ros::Publisher normal_labels_pub_;
        ros::Publisher path_pub_;
        ros::Publisher minpos_path_pub_;
        ros::Subscriber cloud_sub_;

        std::string position_name_;
        std::string normal_name_;

        std::string map_frame_;
        std::string robot_frame_;
        float max_cloud_age_;
        float max_pitch_;
        float max_roll_;

//        float robot_size_;
        int neighborhood_knn_;
        float neighborhood_radius_;

        int queue_size_;

//        typedef std::lock_guard<std::recursive_mutex> Guard;
        std::recursive_mutex cloud_mutex_;
        sensor_msgs::PointCloud2::ConstPtr cloud_;
    };

}  // namespace naex

#endif //NAEX_PLANNER_H
