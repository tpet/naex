//
// Created by petrito1 on 11/18/20.
//

#ifndef NAEX_NAEX_H
#define NAEX_NAEX_H

#include <memory>
#include <naex/clouds.h>
#include <nearest_neighbors.h>
#include <naex/types.h>
#include <nav_msgs/GetPlanRequest.h
#include <sensor_msgs/point_cloud2_iterator.h>
#include <unordered_set>

// NB: FLANN knnSearch is not thread-safe!
// https://stackoverflow.com/questions/39257005/is-opencvflannindex-knnsearch-thread-safe/39339909

// NB: Normal radius should be smaller than robot radius so that normals are not
// skewed in traversable points.

namespace naex
{
    class Exception: public std::runtime_error
    {
    public:
        NaexException(const char* what):
            std::runtime_error(what)
        {}
    };

    class NotInitialized: public Exception
    {
    public:
        NotInitialized(const char* what):
            Exception(what)
        {}
    };

    // Not needed, a contiguous buffer within NN graph will be better.
    /*
    template<typename Index, typename Elem>
    class PointSets
    {
    public:
        void add_points();
        Elem* get_point();
        Elem* remove_point(Index i);
        void prune_invalid();
//        std::map<Buffer<Elem>> x_buf_;
//        std::vector<flann::Matrix<Elem>> x_;
        std::vector<Index> pt_to_set_;
        std::vector<Index> set_to_start_pt_;
        std::vector<bool> valid_;
    };
    */

    class Parameters
    {
    public:
        Parameters():
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
            normal_radius_(.5),
            max_nn_height_diff_(.15),
            viewpoints_update_freq_(1.),
            viewpoints_(),
            clearance_low_(.15),
            clearance_high_(.8),
            min_points_obstacle_(3),
            max_ground_diff_std_(.1),
            max_ground_abs_diff_mean_(.1),
            edge_min_centroid_offset_(.75),
            min_dist_to_obstacle_(.7),
            other_viewpoints_(),
            min_vp_distance_(1.5),
            max_vp_distance_(5.),
            self_factor_(.5),
            planning_freq_(1.),
            num_input_clouds_(1),
            queue_size_(5),

            default_request_( )
        {}

        void load_ros_params(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        {
            std::lock_guard<std::recursive_mutex> lock(mutex_);

            pnh.param("position_name", position_name_, position_name_);
            pnh.param("normal_name", normal_name_, normal_name_);
            pnh.param("map_frame", map_frame_, map_frame_);
            pnh.param("robot_frame", robot_frame_, robot_frame_);
            pnh.param("robot_frames", robot_frames_, robot_frames_);
            pnh.param("max_cloud_age", max_cloud_age_, max_cloud_age_);
            pnh.param("max_pitch", max_pitch_, max_pitch_);
            pnh.param("max_roll", max_roll_, max_roll_);
            pnh.param("neighborhood_knn", neighborhood_knn_, neighborhood_knn_);
            pnh.param("neighborhood_radius", neighborhood_radius_, neighborhood_radius_);
            pnh.param("min_normal_pts", min_normal_pts_, min_normal_pts_);
            pnh.param("normal_radius", normal_radius_, normal_radius_);

            pnh.param("max_nn_height_diff", max_nn_height_diff_, max_nn_height_diff_);
            pnh.param("clearance_low", clearance_low_, clearance_low_);
            pnh.param("clearance_high", clearance_high_, clearance_high_);
            pnh.param("min_points_obstacle", min_points_obstacle_, min_points_obstacle_);
            pnh.param("max_ground_diff_std", max_ground_diff_std_, max_ground_diff_std_);
            pnh.param("max_ground_abs_diff_mean", max_ground_abs_diff_mean_, max_ground_abs_diff_mean_);
            pnh.param("edge_min_centroid_offset", edge_min_centroid_offset_, edge_min_centroid_offset_);
            pnh.param("min_dist_to_obstacle", min_dist_to_obstacle_, min_dist_to_obstacle_);

            pnh.param("viewpoints_update_freq", viewpoints_update_freq_, viewpoints_update_freq_);
            pnh.param("min_vp_distance", min_vp_distance_, min_vp_distance_);
            pnh.param("max_vp_distance", max_vp_distance_, max_vp_distance_);
            pnh.param("self_factor", self_factor_, self_factor_);
            pnh.param("planning_freq", planning_freq_, planning_freq_);

            pnh.param("num_input_clouds", num_input_clouds_, num_input_clouds_);
            pnh.param("input_queue_size", queue_size_, queue_size_);
            pnh.param("points_min_dist", map_.points_min_dist_, map_.points_min_dist_);
            pnh.param("min_empty_cos", map_.min_empty_cos_, map_.min_empty_cos_);
            pnh.param("empty_ratio", empty_ratio_, empty_ratio_);
            pnh.param("filter_robots", filter_robots_, filter_robots_);

            default_request_.start.pose.position.x = std::numeric_limits<double>::quiet_NaN();
            default_request_.start.pose.position.y = std::numeric_limits<double>::quiet_NaN();
            default_request_.start.pose.position.z = std::numeric_limits<double>::quiet_NaN();
            default_request_.goal.pose.position.x = std::numeric_limits<double>::quiet_NaN();
            default_request_.goal.pose.position.y = std::numeric_limits<double>::quiet_NaN();
            default_request_.goal.pose.position.z = std::numeric_limits<double>::quiet_NaN();
            default_request_.tolerance = 32.;
            pnh.param("start/pose/position/x", default_request_.start.pose.position.x, default_request_.start.pose.position.x);
            pnh.param("start/pose/position/y", default_request_.start.pose.position.y, default_request_.start.pose.position.y);
            pnh.param("start/pose/position/z", default_request_.start.pose.position.z, default_request_.start.pose.position.z);
            pnh.param("goal/pose/position/x", default_request_.goal.pose.position.x, default_request_.goal.pose.position.x);
            pnh.param("goal/pose/position/y", default_request_.goal.pose.position.y, default_request_.goal.pose.position.y);
            pnh.param("goal/pose/position/z", default_request_.goal.pose.position.z, default_request_.goal.pose.position.z);
            pnh.param("tolerance", default_request_.tolerance, default_request_.tolerance);

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
                robot_frames_["self"] = robot_frame_;
            }
        }

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

        float viewpoints_update_freq_;

        float min_vp_distance_;
        float max_vp_distance_;
        float self_factor_;
        float planning_freq_;

        int num_input_clouds_;
        int queue_size_;

        std::recursive_mutex mutex_;

        nav_msgs::GetPlanRequest default_request_;
    };

    class Map
    {
    public:
        Map()
//        :
//            cloud_(),
//            modifier_(cloud_)
        {
//            cloud_.
        }

        void initialize_cloud()
        {
            append_position_fields(cloud_);
            append_normal_fields(cloud_);
            append_occupancy_fields(cloud_);
            append_traversability_fields(cloud_);
            append_planning_fields(cloud_);
        }

//        enum MapField
//        {
//            POSITION = 1,
//        };
//        template<typename It>
//        void insert_points(It iter);  // iter[0], iter[1], iter[2]
        /**
         * Update occupancy, remove dynamic points, update map with new points.
         *
         * Occupancy update:
         * (1) based on projection model, no index -> faster,
         * (2) based on NN directions, need to create index -> slower, may be
         * more precise.
         *
         * Dynamic points:
         * Remove x if
         * (1) empty(x) - occupied(x) > thresh
         * (2) empty(X) / occupied(x) > thresh
         *
         * New points:
         * Add point if far enough from current measurements.
         *
         * Index update:
         * Fill in new points.
         * Add points to index with flann matrix wrapper.
         *
         * @param cloud
         */
        void insert_cloud(const sensor_msgs::PointCloud2& cloud)
        {
//            SphericalProjection proj;
//            proj.fit(cloud);
//            proj.check(cloud);

        }
//        void iterator(FieldId fid);

        void resize(size_t n)
        {
            modifier_.resize(n);
        }

        void reserve(size_t n)
        {
            modifier_.resize(n);
        }

        flann::Index index_;  // FLANN point index.
        void add_points();
        Type* get_point();
        Type* remove_point(Index i);
        void update_valid_neighbors();

//        PointCloud2 cloud_;
//        std::vector<sensor_msgs::PointCloud2Iterator> iters_;
//        Buffer<uint8_t> data_;

        // PointCloud2 provides convenient multi-channel cloud representation
        // with arbitrary types and field iterators.
        sensor_msgs::PointCloud2 cloud_;
        sensor_msgs::PointCloud2Modifier modifier_;
        // NB: Iterators are initialized only once. Avoid using iter.end() as
        //   this won't be valid!
        sensor_msgs::PointCloud2Iterator<Value> x_iter_;
        // NN Graph
        sensor_msgs::PointCloud2Iterator<Index> nn_count_iter_;
        sensor_msgs::PointCloud2Iterator<Index> nn_iter_;
        // Viewpoint (for occupancy assessment and measurement distance)
        sensor_msgs::PointCloud2Iterator<Value> vp_x_iter_;
        // Geometric features
//        sensor_msgs::PointCloud2Iterator<Value> normal_x_iter_;
        sensor_msgs::PointCloud2Iterator<int8_t> normal_x_iter_;
        // Normal scale is common
//        sensor_msgs::PointCloud2Iterator<Value> normal_scale_iter_;
        sensor_msgs::PointCloud2Iterator<uint8_t> normal_support_iter_;
        // Occupancy
        sensor_msgs::PointCloud2Iterator<uint8_t> empty_iter_;
        sensor_msgs::PointCloud2Iterator<uint8_t> occupied_iter_;
        // Traversability labels
        sensor_msgs::PointCloud2Iterator<uint8_t> normal_label_iter_;
        sensor_msgs::PointCloud2Iterator<uint8_t> func_label_iter_;
        // Planning costs and rewards
        sensor_msgs::PointCloud2Iterator<Value> path_cost_iter_;
//        sensor_msgs::PointCloud2Iterator<Value> min_vp_dist_;
//        sensor_msgs::PointCloud2Iterator<Value> min_other_vp_dist_;
        sensor_msgs::PointCloud2Iterator<Value> reward_iter_;
        sensor_msgs::PointCloud2Iterator<Value> rel_cost_iter_;
        // Index metadata
        // unindexed
        // indexed
        // to_update
        sensor_msgs::PointCloud2Iterator<uint8_t> index_state_iter_;
    };

    class Plan
    {
    public:
        Plan(size_t n);
        Buffer<Index> pred_;
        Buffer<Cost> cost_;
    };

    class Planner
    {
    public:
        typedef std::recursive_mutex RMutex;
        typedef std::lock_guard<RMutex> RLock;

        Planner(ros::NodeHandle& nh, ros::NodeHandle& pnh)
        {
            params_.get_ros_params(nh, pnh);
        }

        void initialize_cloud();
        void resize_cloud();
        void plan(const Map& map, Plan& plan);

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

        std::string map_frame_;
        std::string robot_frame_;
        std::map<std::string, std::string> robot_frames_;

        Mutex viewpoints_mutex_;
        std::vector<Elem> viewpoints_;  // 3xN
        std::vector<Elem> other_viewpoints_;  // 3xN
//        std::map<std::string, Vec3> last_positions_;

        Mutex initialized_mutex_;
        bool initialized_;

        Mutex map_mutex_;
        Map map_;

        Parameters params_;
    };
}

#endif //NAEX_NAEX_H
