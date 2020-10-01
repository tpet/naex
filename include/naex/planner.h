//
// Created by petrito1 on 10/1/20.
//

#ifndef NAEX_PLANNER_H
#define NAEX_PLANNER_H

#include <cmath>
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
            sensor_msgs::PointCloud2ConstIterator<float> iter_nx(cloud, "normal_x");
            const flann::Matrix<float> nx(const_cast<float*>(&iter_nx[0]), n_pts, 3, cloud.point_step);

            // Create NN index.
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud, "x");
            const flann::Matrix<float> x(const_cast<float*>(&iter_x[0]), n_pts, 3, cloud.point_step);
            const flann::Index<flann::L2<float>> index(x, flann::KDTreeIndexParams(4));

            // Construct NN graph.
            std::unique_ptr<float[]> dist_buf(new float[n_pts * neighborhood_knn_]);
            std::unique_ptr<int[]> ind_buf(new int[n_pts * neighborhood_knn_]);
            // Otherwise, one has cast const away from cloud data above.
            flann::Matrix<float> dist(dist_buf.get(), n_pts, neighborhood_knn_);
            flann::Matrix<int> ind(ind_buf.get(), n_pts, neighborhood_knn_);
            flann::SearchParams params;
            params.max_neighbors = neighborhood_knn_;
            index.radiusSearch(x, ind, dist, neighborhood_radius_, params);

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
