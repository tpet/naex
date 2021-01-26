
#include <naex/clouds.h>
#include <naex/timer.h>
#include <ros/ros.h>

namespace naex
{

class LidarModel
{
public:
    LidarModel(ros::NodeHandle& nh, ros::NodeHandle& pnh):
            nh_(nh),
            pnh_(pnh),
            model_(),
            check_model_(false)
    {
        pnh.param("check_model", check_model_, check_model_);
        cloud_sub_ = nh.subscribe("cloud", 2, &LidarModel::cloud_received, this);
    }

    void cloud_received(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        Timer t;
        ROS_DEBUG("Cloud %u-by-%u received.", msg->height, msg->width);
        print_cloud_summary(*msg);
        if (!model_.fit(*msg))
        {
            ROS_WARN("Fitting sensor model failed.");
            return;
        }
        ROS_INFO("Cloud %u-by-%u: "
                 "elevation [%.3g, %.3g], step %.3g, "
                 "azimuth [%.3g, %.3g], step %.3g [deg] "
                 "(%.3f s).",
                 model_.height_, model_.width_,
                 degrees(model_.elevation_start_),
                 degrees(model_.elevation_start_ + (model_.height_ - 1) * model_.elevation_step_),
                 degrees(model_.elevation_step_),
                 degrees(model_.azimuth_start_),
                 degrees(model_.azimuth_start_ + (model_.width_ - 1) * model_.azimuth_step_),
                 degrees(model_.azimuth_step_),
                 t.seconds_elapsed());
        if (check_model_)
        {
            model_.check(*msg);
        }
    }

private:
    ros::NodeHandle& nh_;
    ros::NodeHandle& pnh_;
    ros::Subscriber cloud_sub_;
    SphericalProjection model_;
    bool check_model_;
};

}

int main (int argc, char *argv[])
{
    ros::init(argc, argv, "lidar_model");
    ros::NodeHandle nh, pnh("~");
    naex::LidarModel node(nh, pnh);
    ros::spin();
}
