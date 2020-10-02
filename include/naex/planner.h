//
// Created by petrito1 on 10/1/20.
//

#ifndef NAEX_PLANNER_H
#define NAEX_PLANNER_H

#include <cmath>
#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace naex
{
    geometry_msgs::PoseStamped to_pose(const geometry_msgs::TransformStamped& tf)
    {
        geometry_msgs::PoseStamped pose;
        pose.header = tf.header;
        pose.pose.position.x = tf.transform.translation.x;
        pose.pose.position.y = tf.transform.translation.y;
        pose.pose.position.z = tf.transform.translation.z;
        pose.pose.orientation = tf.transform.rotation;
        return pose;
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

    template<typename V>
    class ValueIterator
    {
    public:
        ValueIterator(const V& val):
                value_(val)
        {}
        ValueIterator& operator++() { value_++; return *this; }
        ValueIterator& operator--() { value_--; return *this; }
        bool operator!=(const ValueIterator<V>& other) { return value_ != other.value_; }
        V& operator*() { return value_; }
        const V& operator*() const { return value_; }
    private:
        V value_;
    };

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

    typedef ValueIterator<Vertex> VertexIter;
    typedef ValueIterator<Edge> EdgeIter;

    class Graph
    {
    public:
        Graph(flann::Matrix<Elem> points, flann::Matrix<Elem> normals):
                points_(points),
                points_index_(points_, flann::KDTreeIndexParams(4)),
                normals_(normals)
        {}

        void compute(Vertex k, Elem radius)
        {
            dist_buf_.reset(new Elem[num_vertices() * k]);
            nn_buf_.reset(new int[num_vertices() * k]);
            dist_ = flann::Matrix<Cost>(dist_buf_.get(), num_vertices(), k);
            nn_ = flann::Matrix<int>(nn_buf_.get(), num_vertices(), k);
            flann::SearchParams params;
            params.max_neighbors = k;
            points_index_.radiusSearch(points_, nn_, dist_, radius, params);
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
            return {0, num_vertices()};
        }

        inline std::pair<EdgeIter, EdgeIter> out_edges(const Vertex& u) const
        {
            // TODO: Limit to valid edges here or just by costs?
            return {u * num_edges(), (u + 1) * num_edges()};
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
            Vec2 xy_dir = Vec2Map(points_[v1]) - Vec2Map(points_[v0]);
            Elem height_diff = points_[v1][2] - points_[v0][2];
            Elem inclination = std::atan(height_diff / xy_dir.norm());
            if (inclination > max_pitch_)
                return std::numeric_limits<Cost>::infinity();
            // TODO: Check pitch and roll separately.
            Elem pitch0 = std::acos(normals_[v0][2]);
            if (pitch0 > max_pitch_)
                return std::numeric_limits<Cost>::infinity();
            Elem pitch1 = std::acos(normals_[v0][2]);
            if (pitch1 > max_pitch_)
                return std::numeric_limits<Cost>::infinity();
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

        // NN and distances
        std::unique_ptr<int[]> nn_buf_;
        std::unique_ptr<Cost[]> dist_buf_;
        flann::Matrix<int> nn_;
        flann::Matrix<Cost> dist_;

        float max_pitch_;
        float max_roll_;
    };

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
            cloud_sub_ = nh_.subscribe("cloud", queue_size_, &Planner::cloud_received, this);
        }

        void plan(const sensor_msgs::PointCloud2& cloud, const geometry_msgs::PoseStamped& start)
        {
            const auto n_pts = cloud.height * cloud.width;

            // TODO: Recompute normals.
            // TODO: Build map from all aligned input clouds (interp tf).
            sensor_msgs::PointCloud2ConstIterator<Elem> it_normals(cloud, "normal_x");
            const flann::Matrix<Elem> normals(const_cast<float*>(&it_normals[0]), n_pts, 3, cloud.point_step);

            // Create NN index.
            sensor_msgs::PointCloud2ConstIterator<Elem> it_points(cloud, "x");
            const flann::Matrix<Elem> points(const_cast<Elem*>(&it_points[0]), n_pts, 3, cloud.point_step);

            // Construct NN graph.
            Graph graph(points, normals);
            graph.compute(neighborhood_knn_, neighborhood_radius_);

            // Plan in NN graph with approx. travel time costs.

            // Add penalty for initial rotation: + abs(angle_err) / angular_max.
            // Subtract utility from visiting the frontiers: - observation distance.
        }

        void cloud_received(const sensor_msgs::PointCloud2::ConstPtr& cloud)
        {
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


            auto tf = tf_->lookupTransform(cloud->header.frame_id, robot_frame_, ros::Time::now());

            geometry_msgs::PoseStamped start;
            plan(*cloud, start);
        }

    protected:
        ros::NodeHandle& nh_;
        ros::NodeHandle& pnh_;
        std::unique_ptr<tf2_ros::Buffer> tf_;
        std::unique_ptr<tf2_ros::TransformListener> tf_sub_;
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

}

#endif //NAEX_PLANNER_H
