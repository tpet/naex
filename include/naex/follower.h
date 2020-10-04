//
// Created by petrito1 on 10/1/20.
//

#ifndef NAEX_FOLLOWER_H
#define NAEX_FOLLOWER_H

#include <cmath>
#include <flann/flann.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <mutex>
#include <ros/ros.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/buffer.h>

namespace naex
{

    class Follower
    {
    public:
        Follower(ros::NodeHandle& nh, ros::NodeHandle& pnh)
                :
                nh_(nh),
                pnh_(pnh)
        {
            update_from_params();
        }

        void update_from_params()
        {
            pnh_.param("max_cloud_age", max_cloud_age_, 5.);
            pnh_.param("max_pitch", max_pitch_, 30. / 180. * M_PI);
            pnh_.param("max_roll", max_roll_, 30. / 180. * M_PI);
        }

    protected:
        ros::NodeHandle& nh_;
        ros::NodeHandle& pnh_;

        double max_cloud_age_;
        double max_pitch_;
        double max_roll_;

        std::recursive_mutex cloud_mutex_;
        const sensor_msgs::PointCloud2::ConstPtr cloud_;
//    flann::Matrix<float> points_;
//    flann::Index<flann::L2<float>> point_index_;
//    flann::Matrix<float> normals_;
    };

}

#endif //NAEX_FOLLOWER_H
