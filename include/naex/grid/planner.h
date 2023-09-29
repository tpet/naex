#pragma once

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <naex/grid/graph.h>
#include <naex/grid/grid.h>
#include <naex/grid/search.h>
#include <naex/iterators.h>
#include <naex/timer.h>
#include <naex/transforms.h>
#include <naex/types.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <unordered_map>

namespace naex
{
namespace grid
{

template<typename T>
std::string format(T x, T y, T z)
{
    std::stringstream s;
    s << "(" << x << ", " << y << ", " << z << ")";
    return s.str();
}
std::string format(const geometry_msgs::Vector3& v)
{
    return format(v.x, v.y, v.z);
}
std::string format(const geometry_msgs::Point& v)
{
    return format(v.x, v.y, v.z);
}
std::string format(const Vec3& v)
{
    return format(v.x(), v.y(), v.z());
}

Vec3 toVec3(const geometry_msgs::Point& p)
{
    return Vec3(p.x, p.y, p.z);
}
Vec3 toVec3(const geometry_msgs::Vector3& v)
{
    return Vec3(v.x, v.y, v.z);
}
Vec3 toVec3(const Point2f& v)
{
    return Vec3(v.x, v.y, 0.f);
}

void tracePathVertices(VertexId v0,
                       VertexId v1,
                       const std::vector<VertexId>& predecessor,
                       std::vector<VertexId>& path_vertices)
{
    assert(predecessor[v0] == v0);
    Vertex v = v1;
    while (v != v0)
    {
        path_vertices.push_back(v);
        v = predecessor[v];
    }
    path_vertices.push_back(v);
    std::reverse(path_vertices.begin(), path_vertices.end());
}

std::vector<VertexId> tracePathVertices(VertexId v0,
                                        VertexId v1,
                                        const std::vector<VertexId>& predecessor)
{
    std::vector<VertexId> path_vertices;
    tracePathVertices(v0, v1, predecessor, path_vertices);
    return path_vertices;
}

void appendPath(const std::vector<VertexId>& path_vertices,
                const Grid& grid,
                nav_msgs::Path& path)
{
    if (path_vertices.empty())
    {
        return;
    }
    path.poses.reserve(path.poses.size() + path_vertices.size());
    for (const auto& v: path_vertices)
    {
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = grid.point(v).x;
        pose.pose.position.y = grid.point(v).y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.w = 1.;
        if (!path.poses.empty())
        {
            Vec3 x(pose.pose.position.x - path.poses.back().pose.position.x,
                   pose.pose.position.y - path.poses.back().pose.position.y,
                   pose.pose.position.z - path.poses.back().pose.position.z);
            x.normalize();
            Vec3 z(0, 0, 1);
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

template<typename T>
bool isValid(T x, T y, T z)
{
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}
bool isValid(const Vec3& p)
{
    return isValid(p.x(), p.y(), p.z());
}
bool isValid(const geometry_msgs::Point& p)
{
    return isValid(p.x, p.y, p.z);
}
bool isValid(const geometry_msgs::Vector3& p)
{
    return isValid(p.x, p.y, p.z);
}

/**
 * @brief Global planner on 2D grid.
 * 
 * It uses multi-level grid from multiple sources.
 * The first level may be constructed from a map and remain static.
 * The second level may be dynamic, updated from external traversability.
 */
class Planner
{
public:
    Planner(ros::NodeHandle& nh, ros::NodeHandle& pnh):
        nh_(nh),
        pnh_(pnh)
    {
        // Invalid position invokes exploration mode.
        last_request_.start.pose.position.x = std::numeric_limits<double>::quiet_NaN();
        last_request_.start.pose.position.y = std::numeric_limits<double>::quiet_NaN();
        last_request_.start.pose.position.z = std::numeric_limits<double>::quiet_NaN();
        last_request_.goal.pose.position.x = std::numeric_limits<double>::quiet_NaN();
        last_request_.goal.pose.position.y = std::numeric_limits<double>::quiet_NaN();
        last_request_.goal.pose.position.z = std::numeric_limits<double>::quiet_NaN();
        last_request_.tolerance = 2.0f;

        pnh_.param("position_field", position_field_, position_field_);
        pnh_.param("cost_fields", cost_fields_, cost_fields_);
        pnh_.param("map_frame", map_frame_, map_frame_);
        pnh_.param("robot_frame", robot_frame_, robot_frame_);
        pnh_.param("tf_timeout", tf_timeout_, tf_timeout_);

        pnh_.param("max_cloud_age", max_cloud_age_, max_cloud_age_);
        pnh_.param("input_range", input_range_, input_range_);

        float cell_size = 1.0;
        pnh_.param("cell_size", cell_size, cell_size);
        float forget_factor = 1.0;
        pnh_.param("forget_factor", forget_factor, forget_factor);
        grid_ = Grid(cell_size, forget_factor);

        // 4 or 8
        pnh_.param("neighborhood", neighborhood_, neighborhood_);
        std::vector<float> max_costs;
        pnh_.param("max_costs", max_costs, max_costs);
        max_costs_ = max_costs;
        pnh_.param("min_path_cost", min_path_cost_, min_path_cost_);
        pnh_.param("planning_freq", planning_freq_, planning_freq_);
        pnh_.param("plan_from_goal_dist", plan_from_goal_dist_, plan_from_goal_dist_);

        int num_input_clouds = 1;
        pnh_.param("num_input_clouds", num_input_clouds, num_input_clouds);
        num_input_clouds = std::max(1, num_input_clouds);
        int queue_size = 2;
        pnh_.param("input_queue_size", queue_size, queue_size);
        queue_size = std::max(1, queue_size);

        tf_ = std::make_shared<tf2_ros::Buffer>();
        tf_sub_ = std::make_shared<tf2_ros::TransformListener>(*tf_);

        map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("map", 2);
        local_map_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("local_map", 2);
        path_pub_ = nh_.advertise<nav_msgs::Path>("path", 2);

        for (int i = 0; i < num_input_clouds; ++i)
        {
            std::stringstream ss;
            ss << "input_cloud_" << i;
            input_cloud_subs_.push_back(nh_.subscribe<sensor_msgs::PointCloud2>(ss.str(), queue_size,
                boost::bind(&Planner::receiveCloudSafe, this, _1, i)));
        }

        if (planning_freq_ > 0.f)
        {
            planning_timer_ = nh_.createTimer(ros::Rate(planning_freq_),
                                              &Planner::planningTimer, this);
            ROS_INFO("Re-plan automatically at %.1f Hz using the last request.",
                     planning_freq_);
        }
        else
        {
            ROS_WARN("Don't re-plan automatically using the last request.");
        }

        get_plan_service_ = nh_.advertiseService("get_plan", &Planner::planSafe, this);

        ROS_INFO("Node initialized.");
    }

    bool plan(nav_msgs::GetPlanRequest& req, nav_msgs::GetPlanResponse& res)
    {
        if (grid_.empty())
        {
            ROS_WARN("Cannot plan in empty grid.");
            return false;
        }

        Timer t;
        Timer t_part;
        ROS_INFO("Planning request from %s to %s with tolerance %.1f m.",
                 format(req.start.pose.position).c_str(),
                 format(req.goal.pose.position).c_str(),
                 req.tolerance);
        {
            last_request_ = req;
        }
        
        geometry_msgs::PoseStamped start = req.start;
        if (!isValid(start.pose.position))
        {
            const auto tf = tf_->lookupTransform(
                map_frame_, robot_frame_, ros::Time(0), ros::Duration(tf_timeout_));
            transform_to_pose(tf, start);
        }

        Vec3 goal_position = toVec3(req.goal.pose.position);

        // TODO: Use the nearest traversable point to robot as the starting point.
        Vec3 p0 = toVec3(start.pose.position);
        
        const VertexId v0 = grid_.cellId(grid_.pointToCell({p0.x(), p0.y()}));
        
        ShortestPaths sp(grid_, v0, neighborhood_, max_costs_);
        ROS_INFO("Dijkstra (%lu pts): %.3f s.", grid_.size(), t_part.seconds_elapsed());
        createAndPublishMapCloud(sp);

        // If planning for a given goal, return path to the closest reachable
        // point from the goal.
        t_part.reset();
        if (isValid(req.goal.pose.position))
        {
            Vec3 p1 = toVec3(req.goal.pose.position);
            p1.z() = 0.f;

            VertexId v1 = INVALID_VERTEX;
            Value best_dist = std::numeric_limits<Cost>::infinity();
            // TODO: Use graph vertex iterator.
            for (VertexId v = 0; v < grid_.size(); ++v)
            {
                if (!std::isfinite(sp.pathCost(v)))
                {
                    continue;
                }

                Value dist = (toVec3(grid_.point(v)) - p1).norm();
                if (dist < best_dist)
                {
                    v1 = v;
                    best_dist = dist;
                }
            }
            if (v1 == INVALID_VERTEX)
            {
                ROS_ERROR("No feasible path towards %s was found (%.6f, %.3f s).",
                          format(goal_position).c_str(),
                          t_part.seconds_elapsed(), t.seconds_elapsed());
                return false;
            }
            auto path_vertices = tracePathVertices(v0, v1, sp.predecessors());
            res.plan.header.frame_id = map_frame_;
            res.plan.header.stamp = ros::Time::now();
            res.plan.poses.push_back(start);
            appendPath(path_vertices, grid_, res.plan);
            ROS_INFO("Path with %lu poses toward goal %s planned (%.3f s).",
                     res.plan.poses.size(),
                     format(goal_position).c_str(),
                     t.seconds_elapsed());
            return true;
        }

        // TODO: Return random path in exploration mode.
        return false;
    }

    void fillMapCloud(sensor_msgs::PointCloud2& cloud,
                      const Grid& grid,
                      const std::vector<Cost>& path_costs)
    {
        append_field<float>("x", 1, cloud);
        append_field<float>("y", 1, cloud);
        append_field<float>("z", 1, cloud);
        append_field<float>("cost", 1, cloud);
        append_field<float>("path_cost", 1, cloud);
        resize_cloud(cloud, 1, grid_.size());

        sensor_msgs::PointCloud2Iterator<float> x_it(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> cost_it(cloud, "cost");
        sensor_msgs::PointCloud2Iterator<float> path_cost_it(cloud, "path_cost");
        for (VertexId v = 0; v < grid_.size(); ++v, ++x_it, ++cost_it, ++path_cost_it)
        {
            const auto p = grid_.point(v);
            x_it[0] = p.x;
            x_it[1] = p.y;
            x_it[2] = 0.f;
            cost_it[0] = grid_.costs(v).total();
            path_cost_it[0] = path_costs[v];
        }
    }

    void createAndPublishMapCloud(const ShortestPaths& sp)
    {
        sensor_msgs::PointCloud2 cloud;
        cloud.header.frame_id = map_frame_;
        cloud.header.stamp = ros::Time::now();
        fillMapCloud(cloud, grid_, sp.pathCosts());
        map_pub_.publish(cloud);
    }

    bool planSafe(nav_msgs::GetPlanRequest& req, nav_msgs::GetPlanResponse& res)
    {
        try
        {
            return plan(req, res);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_ERROR("Transform failed: %s.", ex.what());
            return false;
        }
    }

    void planningTimer(const ros::TimerEvent& event)
    {
        ROS_DEBUG("Planning timer callback.");
        Timer t;
        nav_msgs::GetPlanRequest req;
        {
            req = last_request_;
        }
        nav_msgs::GetPlanResponse res;
        if (!planSafe(req, res))
        {
            return;
        }
        path_pub_.publish(res.plan);
        ROS_INFO("Planning robot %s path (%lu poses) in map %s: %.3f s.",
                 robot_frame_.c_str(), res.plan.poses.size(), map_frame_.c_str(), t.seconds_elapsed());
    }

    void receiveCloud(const sensor_msgs::PointCloud2::ConstPtr& input, uint8_t level)
    {
        const auto age = (ros::Time::now() - input->header.stamp).toSec();
        if (age > max_cloud_age_)
        {
            ROS_INFO("Skipping old input cloud from %s, age %.1f s > %.1f s.",
                     input->header.frame_id.c_str(), age, max_cloud_age_);
            return;
        }

        geometry_msgs::TransformStamped cloud_to_map;
        cloud_to_map = tf_->lookupTransform(map_frame_, input->header.frame_id, input->header.stamp,
                                            ros::Duration(tf_timeout_));

        Eigen::Isometry3f transform(tf2::transformToEigen(cloud_to_map.transform));

        sensor_msgs::PointCloud2ConstIterator<float> x_it(*input, position_field_);
        sensor_msgs::PointCloud2ConstIterator<float> cost_it(*input,
            level < cost_fields_.size() ? cost_fields_[level].c_str() : "cost");

        for (int i = 0; i < input->height * input->width; ++i, ++x_it, ++cost_it)
        {
            Vec3 p(x_it[0], x_it[1], x_it[2]);
            p = transform * p;
            grid_.updatePointCost({p.x(), p.y()}, level, cost_it[0]);
        }
    }

    void receiveCloudSafe(const sensor_msgs::PointCloud2::ConstPtr& input, uint8_t level)
    {
        try
        {
            receiveCloud(input, level);
        }
        catch (const tf2::TransformException& ex)
        {
            ROS_ERROR("Could not transform input cloud from %s to %s: %s.",
                      input->header.frame_id.c_str(), map_frame_.c_str(), ex.what());
            return;
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
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Timer planning_timer_;

    // Transforms and frames
    std::shared_ptr<tf2_ros::Buffer> tf_{};
    std::shared_ptr<tf2_ros::TransformListener> tf_sub_;
    float tf_timeout_{3.0};
    std::string map_frame_{"map"};
    std::string robot_frame_{"base_footprint"};

    // Publishers
    ros::Publisher map_pub_;
    ros::Publisher updated_map_pub_;
    ros::Publisher local_map_pub_;
    ros::Publisher path_pub_;

    // Subscribers
    std::vector<ros::Subscriber> input_cloud_subs_;

    // Services
    ros::ServiceServer get_plan_service_;
    nav_msgs::GetPlanRequest last_request_;

    // Input
    std::string position_field_{"x"};
    std::vector<std::string> cost_fields_;
    float max_cloud_age_{5.0};
    float input_range_{10.0};

    // Grid
    Grid grid_{};

    // Graph
    int neighborhood_{8};
    Costs max_costs_;

    // Planning
    float min_path_cost_{0.0};
    // Re-planning frequency, repeating the last request if positive.
    float planning_freq_{1.0};
    // Randomize starting vertex within tolerance radius.
    bool random_start_{false};
    double plan_from_goal_dist_{0.0};
    geometry_msgs::PoseStamped last_start_{};
    geometry_msgs::PoseStamped last_goal_{};
};

}  // namespace grid
}  // namespace naex
