//
// Created by petrito1 on 10/1/20.
//

#ifndef NAEX_PLANNER_H
#define NAEX_PLANNER_H

#include <algorithm>
#include <boost/graph/graph_concepts.hpp>
#include <cmath>
#include <cstddef>
#include <Eigen/Dense>
#include <flann/flann.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <map>
#include <mutex>
#include <naex/array.h>
#include <naex/buffer.h>
#include <naex/clouds.h>
#include <naex/exceptions.h>
#include <naex/exclude_frames_filter.h>
#include <naex/flann.h>
#include <naex/iterators.h>
#include <naex/map.h>
#include <naex/nearest_neighbors.h>
#include <naex/range_filter.h>
#include <naex/reward.h>
#include <naex/step_filter.h>
#include <naex/timer.h>
#include <naex/transform_filter.h>
#include <naex/transforms.h>
#include <naex/types.h>
#include <naex/voxel_filter.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <random>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>

namespace naex
{
typedef ValueIterator<Vertex> VertexIter;
typedef ValueIterator<Edge> EdgeIter;
}  // namespace naex

#include <naex/graph.h>

namespace naex
{

class Planner
{
public:
    Planner(ros::NodeHandle& nh, ros::NodeHandle& pnh):
        nh_(nh),
        pnh_(pnh)
    {
        Timer t;
        // Invalid position invokes exploration mode.
        last_request_.start.pose.position.x = std::numeric_limits<double>::quiet_NaN();
        last_request_.start.pose.position.y = std::numeric_limits<double>::quiet_NaN();
        last_request_.start.pose.position.z = std::numeric_limits<double>::quiet_NaN();
        last_request_.goal.pose.position.x = std::numeric_limits<double>::quiet_NaN();
        last_request_.goal.pose.position.y = std::numeric_limits<double>::quiet_NaN();
        last_request_.goal.pose.position.z = std::numeric_limits<double>::quiet_NaN();
        last_request_.tolerance = 2.0f;
        configure();
        ROS_INFO("Initializing. Waiting for other robots...");
        // TODO: Avoid blocking here to be usable as nodelet.
        find_robots(map_frame_, ros::Time(), 15.f);
        Lock lock(initialized_mutex_);
        initialized_ = true;
        time_initialized_ = ros::Time::now().toSec();
        bootstrap_map();
        ROS_INFO("Initialized at %.1f s (%.3f s).",
                 time_initialized_, t.seconds_elapsed());
    }

    void update_params(const ros::WallTimerEvent& evt)
    {
        Timer t;
        pnh_.param("clearance_radius", map_.clearance_radius_, map_.clearance_radius_);
        pnh_.param("clearance_low", map_.clearance_low_, map_.clearance_low_);
        pnh_.param("clearance_high", map_.clearance_high_, map_.clearance_high_);
        pnh_.param("min_points_obstacle", map_.min_points_obstacle_, map_.min_points_obstacle_);
        pnh_.param("max_ground_diff_std", map_.max_ground_diff_std_, map_.max_ground_diff_std_);
        pnh_.param("max_mean_abs_ground_diff", map_.max_mean_abs_ground_diff_, map_.max_mean_abs_ground_diff_);
        pnh_.param("edge_min_centroid_offset", map_.edge_min_centroid_offset_, map_.edge_min_centroid_offset_);
        pnh_.param("min_dist_to_obstacle", map_.min_dist_to_obstacle_, map_.min_dist_to_obstacle_);
        ROS_DEBUG("Parameters updated (%.6f s).", t.seconds_elapsed());
    }

    void configure()
    {
        pnh_.param("position_name", position_name_, position_name_);
        pnh_.param("normal_name", normal_name_, normal_name_);
        pnh_.param("map_frame", map_frame_, map_frame_);
        pnh_.param("robot_frame", robot_frame_, robot_frame_);
//        pnh_.param("robot_frames", robot_frames_, robot_frames_);
        std::map<std::string, std::string> robot_frames;
        pnh_.param("robot_frames", robot_frames, robot_frames);
        robot_frames_.resize(robot_frames.size());
        std::transform(robot_frames.begin(), robot_frames.end(), robot_frames_.begin(),
                       [](const auto& kv) { return kv.second; });

        pnh_.param("max_cloud_age", max_cloud_age_, max_cloud_age_);
        pnh_.param("input_range", input_range_, input_range_);
        pnh_.param("max_pitch", map_.max_pitch_, map_.max_pitch_);
        pnh_.param("max_roll", map_.max_roll_, map_.max_roll_);
        pnh_.param("inclination_penalty", map_.inclination_penalty_, map_.inclination_penalty_);

        pnh_.param("neighborhood_radius", map_.neighborhood_radius_, map_.neighborhood_radius_);
        pnh_.param("normal_radius", normal_radius_, normal_radius_);

        update_params(ros::WallTimerEvent());

        pnh_.param("viewpoints_update_freq", viewpoints_update_freq_, viewpoints_update_freq_);
        pnh_.param("min_vp_distance", min_vp_distance_, min_vp_distance_);
        pnh_.param("max_vp_distance", max_vp_distance_, max_vp_distance_);
        pnh_.param("collect_rewards", collect_rewards_, collect_rewards_);
        pnh_.param("full_coverage_dist", full_coverage_dist_, full_coverage_dist_);
        pnh_.param("coverage_dist_spread", coverage_dist_spread_, coverage_dist_spread_);

        pnh_.param("self_factor", self_factor_, self_factor_);
        pnh_.param("suppress_base_reward", suppress_base_reward_, suppress_base_reward_);
        pnh_.param("path_cost_pow", path_cost_pow_, path_cost_pow_);
        pnh_.param("min_path_cost", min_path_cost_, min_path_cost_);
        pnh_.param("planning_freq", planning_freq_, planning_freq_);
        pnh_.param("random_start", random_start_, random_start_);
        pnh_.param("bootstrap_z", bootstrap_z_, bootstrap_z_);

        int num_input_clouds = 1;
        pnh_.param("num_input_clouds", num_input_clouds, num_input_clouds);
        pnh_.param("input_queue_size", queue_size_, queue_size_);
        pnh_.param("points_min_dist", map_.points_min_dist_, map_.points_min_dist_);

        pnh_.param("min_empty_cos", map_.min_empty_cos_, map_.min_empty_cos_);
        pnh_.param("min_num_empty", map_.min_num_empty_, map_.min_num_empty_);
        pnh_.param("min_empty_ratio", map_.min_empty_ratio_, map_.min_empty_ratio_);
        pnh_.param("max_occ_counter", map_.max_occ_counter_, map_.max_occ_counter_);

        pnh_.param("filter_robots", filter_robots_, filter_robots_);

        bool among_robots = std::find(robot_frames_.begin(), robot_frames_.end(), robot_frame_) != robot_frames_.end();
        if (!among_robots)
        {
            ROS_INFO("Adding robot frame %s to robot frames.", robot_frame_.c_str());
            robot_frames_.push_back(robot_frame_);
        }
        for (const auto& f: robot_frames_)
        {
            ROS_INFO("Robot frame: %s", f.c_str());
        }

        viewpoints_.reserve(size_t(7200. * viewpoints_update_freq_) * 3);
        other_viewpoints_.reserve(size_t(7200. * viewpoints_update_freq_) * 3 * robot_frames_.size());

        tf_ = std::make_shared<tf2_ros::Buffer>(ros::Duration(30.0));
        tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

        viewpoints_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("viewpoints", 5);
        other_viewpoints_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("other_viewpoints", 5);
        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 5);
        updated_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("updated_map", 5);
        dirty_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("dirty_map", 5);
        map_diff_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map_diff", 5);
        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("local_map", 5);
        path_pub_ = nh_.advertise<nav_msgs::Path>("path", 5);

        cloud_sub_ = nh_.subscribe("input_map", queue_size_, &Planner::cloud_received, this);
        for (int i = 0; i < num_input_clouds; ++i)
        {
            std::stringstream ss;
            ss << "input_cloud_" << i;
//            auto sub = nh_.subscribe(ss.str(), queue_size_, &Planner::input_cloud_received, this);
            auto sub = nh_.subscribe(ss.str(), queue_size_, &Planner::input_cloud_received_safe, this);
            input_cloud_subs_.push_back(sub);
        }

        viewpoints_update_timer_ = nh_.createTimer(ros::Rate(viewpoints_update_freq_),
                                                   &Planner::gather_viewpoints, this);
        if (planning_freq_ > 0.f)
        {
            planning_timer_ = nh_.createTimer(ros::Rate(planning_freq_),
                                              &Planner::planning_timer_cb, this);
            ROS_INFO("Re-plan automatically at %.1f Hz using the last request.",
                     planning_freq_);
        }
        else
        {
            ROS_WARN("Don't re-plan automatically using the last request.");
        }

        update_params_timer_ = nh_.createWallTimer(ros::WallDuration(2.0),
                                                   &Planner::update_params, this);

        get_plan_service_ = nh_.advertiseService("get_plan", &Planner::plan, this);
    }

    void bootstrap_map()
    {
        if (std::isnan(bootstrap_z_))
        {
            ROS_WARN("Map not bootstrapped (invalid z).");
            return;
        }
        ROS_INFO("Bootstrapping map with traversable robot neighborhood.");

        int n = int(4 * map_.clearance_radius_ / map_.points_min_dist_ + 1);
        int n_pts = n * n;
        auto now = ros::Time::now();
        auto latest = ros::Time(0);

        Eigen::Isometry3f robot_to_map;
        try
        {
            ros::Duration timeout(15.);
            const auto cloud_to_map_tf = tf_->lookupTransform(map_frame_, robot_frame_, latest, timeout);
            robot_to_map = tf2::transformToEigen(cloud_to_map_tf.transform).cast<float>();
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_ERROR("Could not bootstrap map due to missing transform from %s into map %s: %s.",
                      robot_frame_.c_str(), map_frame_.c_str(), ex.what());
            return;
        }
        ROS_INFO("Position of %s in map %s: [%.1f %.1f %.1f].",
                 robot_frame_.c_str(), map_frame_.c_str(),
                 robot_to_map(0, 3), robot_to_map(1, 3), robot_to_map(2, 3));

        sensor_msgs::PointCloud2 cloud;
        cloud.header.frame_id = map_frame_;
        cloud.header.stamp = now;
        cloud.is_bigendian = bigendian();
        cloud.is_dense = true;
        append_field<float>("x", 1, cloud);
        append_field<float>("y", 1, cloud);
        append_field<float>("z", 1, cloud);
        resize_cloud(cloud, uint32_t(1), uint32_t(n_pts));
        sensor_msgs::PointCloud2Iterator<float> pt(cloud, "x");

        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j, ++pt)
            {
                Vec3 x_cloud((-n / 2 + i) * map_.points_min_dist_,
                             (-n / 2 + j) * map_.points_min_dist_,
                             bootstrap_z_);
                Vec3 x_map = robot_to_map * x_cloud;
                pt[0] = x_map(0);
                pt[1] = x_map(1);
                pt[2] = x_map(2);
            }
        }
        input_map_received(cloud);
        ROS_WARN("Map bootstrapped with %i input points (%lu in map).",
                 n_pts, map_.size());
    }

    Value inline time_from_init(const double time)
    {
        return Value(time - time_initialized_);
    }

    Value inline time_from_init(const ros::Time time)
    {
        return time_from_init(time.toSec());
    }

    void gather_viewpoints(const ros::TimerEvent& event)
    {
        ROS_DEBUG("Gathering viewpoints for %lu actors.", robot_frames_.size());
        Timer t;
        if (map_frame_.empty())
        {
            ROS_ERROR("Could not gather robot positions due to missing map frame.");
            return;
        }
        // TODO: Gathering viewpoints are not necessary. Drop it?
        Lock lock(viewpoints_mutex_);
        for (const auto& frame: robot_frames_)
        {
//            const auto& frame = kv.second;
            try
            {
                // Don't wait for the transforms.
                // Get last transform available.
//                    auto tf = tf_->lookupTransform(map_frame_, frame, ros::Time());
                // Get current time for this actor, past time for other
                // actors to account for transmission time of shared
                // localization.

//                const auto time = (frame == robot_frame_)
//                                  ? ros::Time(std::max(event.current_expected.toSec() - 1., 0.))
//                                  : ros::Time(std::max(event.current_expected.toSec() - 2., 0.));
//                const auto tf = tf_->lookupTransform(map_frame_, frame, time, ros::Duration(0.));

                // Try to get most recent viewpoints.
                const auto time = event.current_expected;
                const auto timeout = std::max(3.0 - (ros::Time::now() - event.current_expected).toSec(), 0.0);
                const auto tf = tf_->lookupTransform(map_frame_, frame, time, ros::Duration(timeout));

                Vec3 pos(Value(tf.transform.translation.x),
                         Value(tf.transform.translation.y),
                         Value(tf.transform.translation.z));

                bool self = (frame == robot_frame_);
                if (self)
                {
                    viewpoints_.push_back(pos);
                }
                else
                {
                    other_viewpoints_.push_back(pos);
                }

                if (map_.empty())
                {
                    ROS_WARN("Empty map, no points updated from gathered viewpoints.");
                    continue;
                }
                Lock cloud_lock(map_.cloud_mutex_);
                Lock index_lock(map_.index_mutex_);

                if (collect_rewards_)
                {
                    RadiusQuery<Value> q1(*map_.index_, FMat(pos.data(), 1, 3), 2 * max_vp_distance_);

                    std::vector<Vec3> vps = {pos};
                    update_coverage(map_.cloud_, q1.nn_[0], vps,
                                    full_coverage_dist_, coverage_dist_spread_, max_vp_distance_,
                                    true, self);

                    collect_rewards(map_.cloud_, q1.nn_[0],
                                    full_coverage_dist_, coverage_dist_spread_, max_vp_distance_,
                                    self_factor_, suppress_base_reward_);
                }
                else
                {
                    RadiusQuery<Value> q(*map_.index_, FMat(pos.data(), 1, 3), max_vp_distance_);
                    assert(q.nn_.size() == 1);
                    assert(q.dist_.size() == 1);

                    ROS_DEBUG("%lu / %lu points within %.1f m from %s origin.",
                              q.nn_[0].size(), map_.index_->size(), max_vp_distance_, frame.c_str());
                    for (Index i = 0; i < q.nn_[0].size(); ++i)
                    {
                        const Vertex v = q.nn_[0][i];
                        const Value d = std::sqrt(q.dist_[0][i]);
                        const Value t = time_from_init(time);
                        // TODO: Account for time to enable patrolling.
                        if (self)
                        {
                            map_.cloud_[v].dist_to_actor_ = (std::isfinite(map_.cloud_[v].actor_last_visit_)
                                                             ? std::min(map_.cloud_[v].dist_to_actor_, d)
                                                             : d);
                            map_.cloud_[v].actor_last_visit_ = t;
                        }
                        else
                        {
                            map_.cloud_[v].dist_to_other_actors_ = (std::isfinite(map_.cloud_[v].other_actors_last_visit_)
                                                                    ? std::min(map_.cloud_[v].dist_to_other_actors_, d)
                                                                    : d);
                            map_.cloud_[v].other_actors_last_visit_ = t;
                        }
                    }
                }
            }
            catch (const tf2::TransformException& ex)
            {
                ROS_WARN_THROTTLE(5.0, "Viewpoint of %s not updated: %s.",
                                  frame.c_str(), ex.what());
                continue;
            }
        }
        auto now = ros::Time::now();
        if (viewpoints_pub_.getNumSubscribers() > 0)
        {
            sensor_msgs::PointCloud2 vp_cloud;
            flann::Matrix<Elem> vp(viewpoints_.data()->data(), viewpoints_.size(), 3);
            create_xyz_cloud(vp, vp_cloud);
            vp_cloud.header.frame_id = map_frame_;
            vp_cloud.header.stamp = now;
            viewpoints_pub_.publish(vp_cloud);
        }
        if (other_viewpoints_pub_.getNumSubscribers() > 0)
        {
            sensor_msgs::PointCloud2 other_vp_cloud;
            flann::Matrix<Elem> other_vp(other_viewpoints_.data()->data(), other_viewpoints_.size(), 3);
            create_xyz_cloud(other_vp, other_vp_cloud);
            other_vp_cloud.header.frame_id = map_frame_;
            other_vp_cloud.header.stamp = now;
            other_viewpoints_pub_.publish(other_vp_cloud);
        }
        ROS_INFO("Gathering viewpoints for %lu actors done (%.3f s).",
                 robot_frames_.size(), t.seconds_elapsed());
    }

    void trace_path_indices(Vertex start, Vertex goal, const Vertex* predecessor,
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
                     const std::vector<Point>& points,
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
            pose.pose.position.x = points[v].position_[0];
            pose.pose.position.y = points[v].position_[1];
            pose.pose.position.z = points[v].position_[2];
            pose.pose.orientation.w = 1.;
            if (!path.poses.empty())
            {
                Vec3 x(pose.pose.position.x - path.poses.back().pose.position.x,
                       pose.pose.position.y - path.poses.back().pose.position.y,
                       pose.pose.position.z - path.poses.back().pose.position.z);
                x.normalize();
//                    Vec3Map z(normals[v]);
                Vec3 z = ConstVec3Map(points[v].normal_);
                // Fix z direction to be consistent with the previous pose.
                // As we start from the current robot pose with correct z
                // orientation, all following z directions get corrected.
                // Quat q_prev(path.poses.back().pose.orientation.w,
                //         path.poses.back().pose.orientation.x,
                //         path.poses.back().pose.orientation.y,
                //         path.poses.back().pose.orientation.z);
                // Mat3 R_prev;
                // R_prev = q_prev;
                // Vec3 z_prev = R_prev.col(2);
                // if (z.dot(z_prev) < 0.)
                // {
                //     z = -z;
                // }
                // Assume map z points upward.
                if (z.dot(Vec3(0.f, 0.f, 1.f)) < 0.)
                {
                    z = -z;
                }

                Mat3 m;
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
//        std::vector<Elem> vp_copy;
        std::vector<Vec3> vp_copy;
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
//        flann::Matrix<Elem> vp(vp_copy.data(), n_vp, 3);
        flann::Matrix<Elem> vp(vp_copy.data()->data(), n_vp, 3);
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
//        std::vector<Elem> vp_copy;
        std::vector<Vec3> vp_copy;
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
//        flann::Matrix<Elem> vp(vp_copy.data(), n_vp, 3);
        flann::Matrix<Elem> vp(vp_copy.data()->data(), n_vp, 3);
//            flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeIndexParams(2));
        flann::Index<flann::L2_3D<Elem>> vp_index(vp, flann::KDTreeSingleIndexParams());
        vp_index.buildIndex();
        Query<Elem> vp_query(vp_index, points, 1);
        return vp_query.dist_buf_;
    }

    void input_map_received(const sensor_msgs::PointCloud2& cloud)
    {
//        Timer t;
//        const size_t n_pts = cloud.height * cloud.width;
//        sensor_msgs::PointCloud2ConstIterator<Elem> it_points(cloud, position_name_);
//        sensor_msgs::PointCloud2ConstIterator<Elem> it_normals(cloud, normal_name_);
//        Buffer<Elem> points_buf(3 * n_pts);
//        Buffer<Elem> normals_buf(3 * n_pts);
//        auto it_points_dst = points_buf.begin();
//        auto it_normals_dst = normals_buf.begin();
//        for (size_t i = 0; i < n_pts; ++i, ++it_points, ++it_normals)
//        {
//            *it_points_dst++ = it_points[0];
//            *it_points_dst++ = it_points[1];
//            *it_points_dst++ = it_points[2];
//            *it_normals_dst++ = it_normals[0];
//            *it_normals_dst++ = it_normals[1];
//            *it_normals_dst++ = it_normals[2];
//        }
//        flann::Matrix<Elem> points(points_buf.begin(), n_pts, 3);
//        flann::Matrix<Elem> normals(normals_buf.begin(), n_pts, 3);
//        ROS_INFO("Copy of %lu points and normals: %.3f s.", n_pts, t.seconds_elapsed());
        Lock cloud_lock(map_.cloud_mutex_);
        Lock index_lock(map_.index_mutex_);
        Lock dirty_lock(map_.dirty_mutex_);

        map_.cloud_.clear();
        map_.graph_.clear();
        map_.clear_dirty();

        auto points = flann_matrix_view<Value>(const_cast<sensor_msgs::PointCloud2&>(cloud), position_name_, uint32_t(3));
//        auto points = const_flann_matrix_view<Value>(cloud, position_name_, uint32_t(3));
        Vec3 zero(0, 0, 0);
//        Value* origin_ptr = viewpoints_.size() >= 3
//                            ? viewpoints_.data()
//                            : &zero(0);
        Value* origin_ptr = !viewpoints_.empty() ? viewpoints_.data()->data() : zero.data();
        flann::Matrix<Value> origin(origin_ptr, 1, 3);
        map_.initialize(points, origin);
        map_.update_dirty();
        send_local_map(origin_ptr, cloud.header.stamp);
    }

    template<typename T>
    bool valid_point(const T x, const T y, const T z)
    {
        return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
    }

    Value distance_reward(Value distance)
    {
        Value r = std::isfinite(distance) ? distance : max_vp_distance_;
        r = r >= min_vp_distance_ ? r : 0.f;
        r /= max_vp_distance_;
        return r;
    }

    bool plan(nav_msgs::GetPlanRequest& req, nav_msgs::GetPlanResponse& res)
    {
        Timer t;
        Timer t_part;
        {
            Lock lock(initialized_mutex_);
            if (!initialized_)
            {
                ROS_WARN("Won't plan. Waiting for initialization.");
                return false;
            }
        }
        ROS_INFO("Planning from [%.1f, %.1f, %.1f] to [%.1f, %.1f, %.1f] with tolerance %.1f.",
                 req.start.pose.position.x, req.start.pose.position.y, req.start.pose.position.z,
                 req.goal.pose.position.x, req.goal.pose.position.y, req.goal.pose.position.z,
                 req.tolerance);
        {
            Lock lock(last_request_mutex_);
            last_request_ = req;
        }

        geometry_msgs::PoseStamped start = req.start;
        if (!valid_point(start.pose.position.x,
                         start.pose.position.y,
                         start.pose.position.z))
        {
            try
            {
                const auto tf = tf_->lookupTransform(map_frame_, robot_frame_,
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

        Lock cloud_lock(map_.cloud_mutex_);
        Lock index_lock(map_.index_mutex_);

        t.reset();

        const size_t min_map_points = Neighborhood::K_NEIGHBORS;
        if (map_.size() < min_map_points)
        {
            ROS_ERROR("Cannot plan in map with %lu < %lu points.",
                      map_.size(), min_map_points);
            return false;
        }

        // TODO: Deal with occupancy on merging.
        // TODO: Index rebuild incrementally with new points.

        // Create debug cloud for visualization of intermediate results.
        sensor_msgs::PointCloud2 debug_cloud;
//        map_.create_debug_cloud(debug_cloud);
        map_.create_cloud_msg(debug_cloud);
        debug_cloud.header.frame_id = map_frame_;
        // TODO: Refresh on sending?
        debug_cloud.header.stamp = ros::Time::now();
        // Reconstruct original 2D shape.
        debug_cloud.height = 1;
        debug_cloud.width = uint32_t(map_.size());

        // Use the nearest traversable point to robot as the starting point.
        Vec3 start_position(Value(start.pose.position.x),
                            Value(start.pose.position.y),
                            Value(start.pose.position.z));
        Value start_tol = req.tolerance > 0. ? req.tolerance : neighborhood_radius_;
        std::vector<Vertex> traversable;
        {
            Lock lock(map_.cloud_mutex_);
            for (const auto v: map_.nearby_indices(start_position.data(), start_tol))
            {
                if (!(map_.cloud_[v].flags_ & TRAVERSABLE)
                        || (map_.cloud_[v].flags_ & EDGE))
                {
                    continue;
                }
                traversable.push_back(v);
            }
        }
        if (traversable.empty())
        {
            ROS_ERROR("No traversable vertex found within %.1f m from [%.1f, %.1f, %.1f].",
                      start_tol, start_position.x(), start_position.y(), start_position.z());
            return false;
        }
        const Vertex v_start = random_start_
                               ? traversable[std::rand() % traversable.size()]
                               : traversable[0];
        ROS_DEBUG("%s point %lu at [%.1f, %.1f, %.1f] chosen "
                  "from %lu traversable ones within %.1f m "
                  "from start position [%.1f, %.1f, %.1f].",
                  (random_start_ ? "Random" : "Closest"), size_t(v_start),
                  map_.cloud_[v_start].position_[0],
                  map_.cloud_[v_start].position_[1],
                  map_.cloud_[v_start].position_[2],
                  traversable.size(), start_tol,
                  req.start.pose.position.x,
                  req.start.pose.position.y,
                  req.start.pose.position.z);

        // TODO: Append starting pose as a special vertex with orientation dependent edges.
        // Note, that for some worlds and robots, the neighborhood must be quite large to get traversable points.
        // See e.g. X1 @ cave_circuit_practice_01.
        Graph g(map_);
        // Plan in NN graph with approx. travel time costs.
        std::vector<Vertex> predecessor(size_t(g.num_vertices()),
                                        INVALID_VERTEX);
        std::vector<Value> path_costs(size_t(g.num_vertices()),
                                      std::numeric_limits<Value>::infinity());
        EdgeCosts edge_costs(map_);
        boost::typed_identity_property_map<Vertex> index_map;

        t_part.reset();
        // TODO: Stop via exception if needed.
        boost::dijkstra_shortest_paths_no_color_map(g, v_start,
                                                    predecessor.data(), path_costs.data(), edge_costs,
                                                    index_map,
                                                    std::less<Value>(), boost::closed_plus<Value>(),
                                                    std::numeric_limits<Value>::infinity(), Value(0.),
                                                    boost::dijkstra_visitor<boost::null_visitor>());
        ROS_INFO("Dijkstra (%u pts): %.3f s.",
                 g.num_vertices(), t_part.seconds_elapsed());

        // If planning for a given goal, return path to the closest reachable
        // point from the goal.
        t_part.reset();
        if (valid_point(req.goal.pose.position.x,
                        req.goal.pose.position.y,
                        req.goal.pose.position.z))
        {
            Vec3 goal_position(Value(req.goal.pose.position.x),
                               Value(req.goal.pose.position.y),
                               Value(req.goal.pose.position.z));
            Vertex v_goal = INVALID_VERTEX;
            Value best_dist = std::numeric_limits<Value>::infinity();
            for (Index v = 0; v < path_costs.size(); ++v)
            {
                if (!std::isfinite(path_costs[v]))
                {
                    continue;
                }
                Value dist = (ConstVec3Map(map_.cloud_[v].position_) - goal_position).norm();
                if (dist < best_dist)
                {
                    v_goal = v;
                    best_dist = dist;
                }
            }
            if (v_goal == INVALID_VERTEX)
            {
                ROS_ERROR("No feasible path towards [%.1f, %.1f, %.1f] was found (%.6f, %.3f s).",
                          goal_position.x(), goal_position.y(), goal_position.z(),
                          t_part.seconds_elapsed(), t.seconds_elapsed());
                return false;
            }
            std::vector<Vertex> path_indices;
            trace_path_indices(v_start, v_goal, predecessor.data(), path_indices);
            res.plan.header.frame_id = map_frame_;
            res.plan.header.stamp = ros::Time::now();
            res.plan.poses.push_back(start);
            append_path(path_indices, map_.cloud_, res.plan);
            ROS_INFO("Path with %lu poses toward fixed goal [%.1f, %.1f, %.1f] planned "
                     "(t_part.seconds_elapsed(), %.3f s).",
                     res.plan.poses.size(),
                     goal_position.x(), goal_position.y(), goal_position.z(),
                     t.seconds_elapsed());
            return true;
        }

        // TODO: Account for time to enable patrolling (coverage half-life).
        Vertex v_goal = INVALID_VERTEX;
        for (Vertex v = 0; v < path_costs.size(); ++v)
        {
            if (!collect_rewards_)
            {
                map_.cloud_[v].reward_ = std::max(std::min(distance_reward(map_.cloud_[v].dist_to_actor_),
                                                           distance_reward(map_.cloud_[v].other_actors_last_visit_)),
                                                  self_factor_ * distance_reward(map_.cloud_[v].dist_to_actor_));
                map_.cloud_[v].reward_ *= (1 + map_.cloud_[v].num_edge_neighbors_);
                // Decrease rewards in specific areas (staging area).
                // TODO: Ensure correct frame (subt) is used here.
                // TODO: Parametrize the areas.
                suppress_reward(map_.cloud_[v]);
            }

            // Keep original path cost, but discount for relative cost.
//            map_.cloud_[v].path_cost_ = std::isfinite(path_costs[v])
//                                        ? path_costs[v]
//                                        : std::numeric_limits<Value>::quiet_NaN();
            map_.cloud_[v].path_cost_ = path_costs[v];
            map_.cloud_[v].relative_cost_ = std::pow(map_.cloud_[v].path_cost_, path_cost_pow_)
                                            / map_.cloud_[v].reward_;
            // Prefer longer feasible paths, with lowest relative costs.
            if (std::isfinite(map_.cloud_[v].path_cost_)
                && (v_goal == INVALID_VERTEX
                    ||  (map_.cloud_[v_goal].path_cost_ < min_path_cost_
                         && map_.cloud_[v].path_cost_ >= min_path_cost_)
                    || map_.cloud_[v].relative_cost_ < map_.cloud_[v_goal].relative_cost_))
            {
                v_goal = v;
            }
        }

        if (map_pub_.getNumSubscribers() > 0)
        {
            Timer t_send;
            sensor_msgs::PointCloud2 map_cloud;
            map_cloud.header.frame_id = map_frame_;
            map_cloud.header.stamp = ros::Time::now();
//            Lock lock(map_.cloud_mutex_);
            map_.create_cloud_msg(map_cloud);
            map_pub_.publish(map_cloud);
            ROS_DEBUG("Sending map: %.3f s.", t_send.seconds_elapsed());
        }

        if (v_goal == INVALID_VERTEX)
        {
            ROS_ERROR("No valid path/goal found.");
            return false;
        }

        // TODO: Remove inf from path cost for visualization?

        Timer t_path;
        std::vector<Vertex> path_indices;
        trace_path_indices(v_start, v_goal, predecessor.data(), path_indices);
        res.plan.header.frame_id = map_frame_;
        res.plan.header.stamp = ros::Time::now();
        res.plan.poses.push_back(start);
//            append_path(path_indices, points, normals, res.plan);
        append_path(path_indices, map_.cloud_, res.plan);
        ROS_INFO("Path with %lu poses to goal [%.1f, %.1f, %.1f] "
                 "has cost %.3f, reward %.3f, relative cost %.3f (%.3f s).",
                 res.plan.poses.size(),
                 map_.cloud_[v_goal].position_[0],
                 map_.cloud_[v_goal].position_[1],
                 map_.cloud_[v_goal].position_[2],
                 map_.cloud_[v_goal].path_cost_,
                 map_.cloud_[v_goal].reward_,
                 map_.cloud_[v_goal].relative_cost_,
                 t.seconds_elapsed());
        return true;
    }

    void cloud_received(const sensor_msgs::PointCloud2::ConstPtr& cloud)
    {
        ROS_INFO("Cloud received (%u points).", cloud->height * cloud->width);
        {
            Lock lock(initialized_mutex_);
            if (!initialized_)
            {
                ROS_INFO("Skipping input cloud. Waiting for initialization.");
                return;
            }
        }

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

    std::vector<Value> find_robots(const std::string& frame, const ros::Time& stamp, float timeout)
    {
        Timer t;
        std::vector<Value> robots;
        robots.reserve(3 * robot_frames_.size());
//        for (const auto kv: robot_frames_)
        for (const auto& frame: robot_frames_)
        {
//            if (robot_frame_ == kv.second)
            if (frame == robot_frame_)
            {
                continue;
            }
            ros::Duration timeout_duration(std::max(timeout - (ros::Time::now() - stamp).toSec(), 0.));
            geometry_msgs::TransformStamped tf;
            try
            {
//                tf = tf_->lookupTransform(frame, stamp, frame, stamp, map_frame_, timeout_duration);
                tf = tf_->lookupTransform(map_frame_, frame, stamp, timeout_duration);
            }
            catch (const tf2::TransformException& ex)
            {
//                ROS_WARN("Could not get %s pose in %s: %s.",
//                         kv.second.c_str(), frame.c_str(), ex.what());
                ROS_WARN("Could not get %s pose in %s: %s.",
                         frame.c_str(), map_frame_.c_str(), ex.what());
                continue;
            }
            robots.push_back(static_cast<Value>(tf.transform.translation.x));
            robots.push_back(static_cast<Value>(tf.transform.translation.y));
            robots.push_back(static_cast<Value>(tf.transform.translation.z));
//            ROS_INFO("Robot %s found in %s at [%.1f, %.1f, %.1f].", kv.second.c_str(), frame.c_str(),
//                     tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
            ROS_INFO("Robot %s found in %s at [%.1f, %.1f, %.1f].", frame.c_str(), map_frame_.c_str(),
                     tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z);
        }
        ROS_INFO("%lu / %lu robots found in %.3f s (timeout %.3f s).",
                 robots.size() / 3, robot_frames_.size(), t.seconds_elapsed(), timeout);
        return robots;
    }

    void check_initialized()
    {
        Lock lock(initialized_mutex_);
        if (!initialized_)
        {
            throw NotInitialized("Not initialized. Waiting for other robots.");
        }
    }

    void send_cloud(ros::Publisher& pub, const ros::Time& stamp = ros::Time(0), bool force = false)
    {
        if (force || pub.getNumSubscribers() > 0)
        {
            Timer t;
            sensor_msgs::PointCloud2 cloud;
            cloud.header.frame_id = map_frame_;
            cloud.header.stamp = stamp.toNSec() == 0 ? ros::Time::now() : stamp;
            map_.create_cloud_msg(cloud);
            if (cloud.height * cloud.width > 0)
            {
                pub.publish(cloud);
                ROS_DEBUG("Sending cloud %s: %.3f s.", pub.getTopic().c_str(), t.seconds_elapsed());
            }
        }
    }

    void send_map(const ros::Time& stamp = ros::Time(0), bool force = false)
    {
        send_cloud(map_pub_, stamp, force);
    }

    template<typename C>
    void send_cloud(ros::Publisher& pub, const C& indices, const ros::Time& stamp = ros::Time(0), bool force = false)
    {
        if (indices.empty())
            return;
        if (force || pub.getNumSubscribers() > 0)
        {
            Timer t;
            sensor_msgs::PointCloud2 cloud;
            cloud.header.frame_id = map_frame_;
            cloud.header.stamp = stamp.toNSec() == 0 ? ros::Time::now() : stamp;
//            {
//                Lock cloud_lock(map_.cloud_mutex_);
//                Lock index_lock(map_.index_mutex_);
                map_.create_cloud_msg(indices, cloud);
//            }
            pub.publish(cloud);
            ROS_DEBUG("Sending cloud %s: %.3f s.", pub.getTopic().c_str(), t.seconds_elapsed());
        }
    }

    void send_dirty_cloud(const ros::Time& stamp = ros::Time(0), bool force = false)
    {
        if (force || dirty_map_pub_.getNumSubscribers() > 0)
        {
            Lock cloud_lock(map_.cloud_mutex_);
            Lock index_lock(map_.index_mutex_);
            Lock updated_lock(map_.updated_mutex_);
            Lock dirty_lock(map_.dirty_mutex_);
            send_cloud(dirty_map_pub_, map_.dirty_indices_, stamp, force);
        }
    }

    void send_updated_cloud(const ros::Time& stamp = ros::Time(0), bool force = false)
    {
        if (force || updated_map_pub_.getNumSubscribers() > 0)
        {
            Lock cloud_lock(map_.cloud_mutex_);
            Lock index_lock(map_.index_mutex_);
            Lock updated_lock(map_.updated_mutex_);
            send_cloud(updated_map_pub_, map_.updated_indices_, stamp, force);
        }
    }

    void send_local_map(Value* origin, const ros::Time& stamp = ros::Time(0), bool force = false)
    {
        if (force || local_map_pub_.getNumSubscribers() > 0)
        {
            const auto indices = map_.nearby_indices(origin, input_range_);
            send_cloud(local_map_pub_, indices, stamp, force);
        }
    }

    void input_cloud_received(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        const auto age = (ros::Time::now() - input->header.stamp).toSec();
        if (age > max_cloud_age_)
        {
            ROS_INFO("Skipping old input cloud from %s, age %.1f s > %.1f s.",
                     input->header.frame_id.c_str(), age, max_cloud_age_);
            return;
        }

        check_initialized();
        sensor_msgs::PointCloud2 step_filtered;
        StepFilter step_filter(1024, 1024);
        step_filter.filter(*input, step_filtered);

        Timer t_tf;
        double wait = std::max(5.0 - (ros::Time::now() - input->header.stamp).toSec(), 0.0);
        geometry_msgs::TransformStamped cloud_to_map;
        cloud_to_map = tf_->lookupTransform(map_frame_, input->header.frame_id, input->header.stamp,
                                            ros::Duration(wait));
        ROS_DEBUG("Had to wait %.3f s for input cloud transform.", t_tf.seconds_elapsed());

        Eigen::Isometry3f transform(tf2::transformToEigen(cloud_to_map.transform));

        // TODO: Update map occupancy based on reconstructed surface of 2D cloud.
        if (step_filtered.height > 1 && step_filtered.width > 1)
        {
            map_.update_occupancy_projection(step_filtered, cloud_to_map.transform);
        }
        else
        {
            ROS_WARN("Cannot update occupancy using unstructured point cloud.");
        }

        Timer t_filter;
        FilterChain<sensor_msgs::PointCloud2>::Filters filters{
            std::make_shared<VoxelFilter<float, int>>("x", map_.points_min_dist_),
            std::make_shared<RangeFilter<float>>("x", 1.f, input_range_),
            std::make_shared<ExcludeFramesFilter<float>>("x", robot_frames_, 1.f, tf_, ros::Duration(3.0)),
            std::make_shared<FilterFromProcessor<sensor_msgs::PointCloud2>>(
                std::make_shared<TransformProcessor<float>>("x", map_frame_, tf_, ros::Duration(3.0)))
        };
        FilterChain<sensor_msgs::PointCloud2> chain(filters);

        auto cloud = std::make_shared<sensor_msgs::PointCloud2>();
        chain.filter(step_filtered, *cloud);
        ROS_INFO("%lu filters applied (%.3f s).", filters.size(), t_filter.seconds_elapsed());

        Vec3 origin = transform.translation();
        flann::Matrix<Elem> origin_mat(origin.data(), 1, 3);

        const auto points = flann_matrix_view<float>(*cloud, "x", 3);

        Lock cloud_lock(map_.cloud_mutex_);
        Lock index_lock(map_.index_mutex_);
        {
            Lock added_lock(map_.updated_mutex_);
            Lock lock_dirty(map_.dirty_mutex_);
            map_.merge(points, origin_mat);
            map_.update_dirty();
//            ROS_INFO("Input cloud with %u points merged: %.3f s.", n_added, t.seconds_elapsed());
            // TODO: Mark affected map points for update?
            send_dirty_cloud(cloud->header.stamp);
            map_.clear_dirty();
            send_updated_cloud(cloud->header.stamp);
            map_.clear_updated();
        }
        send_local_map(origin.data(), cloud->header.stamp);
        send_map(cloud->header.stamp);
    }

    void input_cloud_received_safe(const sensor_msgs::PointCloud2::ConstPtr& input)
    {
        try
        {
            input_cloud_received(input);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_ERROR("Could not transform input cloud from %s to %s: %s.",
                      input->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
            return;
        }
        catch (const Exception& ex)
        {
            ROS_ERROR("Input cloud processing failed: %s\n%s",
                      ex.what(), ex.stacktrace().c_str());
        }
        catch (const std::runtime_error& ex)
        {
            ROS_ERROR("Input cloud processing failed: %s", ex.what());
        }
        catch (...)
        {
            ROS_ERROR("Input cloud processing failed with an unknown exception.");
        }
    }

protected:
    typedef std::recursive_mutex Mutex;
    typedef std::lock_guard<Mutex> Lock;

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
//    std::unique_ptr<tf2_ros::Buffer> tf_;
    std::shared_ptr<tf2_ros::Buffer> tf_{};
    std::shared_ptr<tf2_ros::TransformListener> tf_sub_;

    ros::Publisher path_pub_;
    ros::Publisher viewpoints_pub_;
    ros::Publisher other_viewpoints_pub_;
    ros::Subscriber cloud_sub_;

    std::vector<ros::Subscriber> input_cloud_subs_;
    ros::Publisher map_pub_;
    ros::Publisher updated_map_pub_;
    ros::Publisher dirty_map_pub_;
    ros::Publisher map_diff_pub_;
    ros::Publisher local_map_pub_;
    ros::Timer planning_timer_;
    ros::ServiceServer get_plan_service_;
    Mutex last_request_mutex_;
    nav_msgs::GetPlanRequest last_request_;
    ros::Timer viewpoints_update_timer_;
    ros::WallTimer update_params_timer_;

    std::string position_name_{"x"};
    std::string normal_name_{"normal_x"};

    std::string map_frame_{"map"};
    std::string robot_frame_{"base_footprint"};
//    std::map<std::string, std::string> robot_frames_;
    std::vector<std::string> robot_frames_{};
    float max_cloud_age_{5.0};
    float input_range_{10.0};
    bool filter_robots_{false};

    int neighborhood_knn_{12};
    float neighborhood_radius_{0.5};
    float normal_radius_{neighborhood_radius_};

    Mutex viewpoints_mutex_;
    float viewpoints_update_freq_{1.0};
//    std::vector<Value> viewpoints_{};  // 3xN
//    std::vector<Value> other_viewpoints_{};  // 3xN
//    std::vector<Vec<Value, 3>> viewpoints_{};
//    std::vector<Vec<Value, 3>> other_viewpoints_{};
    std::vector<Vec3> viewpoints_{};
    std::vector<Vec3> other_viewpoints_{};
    float min_vp_distance_{1.5};
    float max_vp_distance_{6.0};
    bool collect_rewards_{true};
    float full_coverage_dist_{3.0};
    float coverage_dist_spread_{1.5};
    float self_factor_{0.25};
    bool suppress_base_reward_{true};
    float path_cost_pow_{1.0};
    float min_path_cost_{0.0};
    // Re-planning frequency, repeating the last request if positive.
    float planning_freq_{0.5};
    // Randomize starting vertex within tolerance radius.
    bool random_start_{false};
    // Z offset for bootstrap map height
    float bootstrap_z_{0.0};
    Mutex initialized_mutex_;
    bool initialized_{false};
    double time_initialized_{std::numeric_limits<double>::quiet_NaN()};

    int queue_size_{5};
    Mutex map_mutex_;
    Map map_{};
};

}  // namespace naex

#endif  // NAEX_PLANNER_H
