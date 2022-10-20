#include <naex/traversability.h>
#include <naex/timer.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace naex
{

class TraversabilityNodelet: public nodelet::Nodelet
{
protected:
    Traversability proc_;
    std::string fixed_frame_;
    double timeout_;
    tf2_ros::Buffer tf_;
    std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
    ros::Publisher cloud_pub_;
    ros::Subscriber cloud_sub_;
    ~TraversabilityNodelet() override = default;
public:
    void updateParams()
    {
        getPrivateNodeHandle().param("min_z", proc_.min_z_, proc_.min_z_);
        getPrivateNodeHandle().param("max_z", proc_.max_z_, proc_.max_z_);
        getPrivateNodeHandle().param("support_radius", proc_.support_radius_, proc_.support_radius_);
        getPrivateNodeHandle().param("min_support", proc_.min_support_, proc_.min_support_);
        getPrivateNodeHandle().param("inclination_radius", proc_.inclination_radius_, proc_.inclination_radius_);
        getPrivateNodeHandle().param("inclination_weight", proc_.inclination_weight_, proc_.inclination_weight_);
        getPrivateNodeHandle().param("normal_std_weight", proc_.normal_std_weight_, proc_.normal_std_weight_);
        getPrivateNodeHandle().param("clearance_radius", proc_.clearance_radius_, proc_.clearance_radius_);
        getPrivateNodeHandle().param("clearance_low", proc_.clearance_low_, proc_.clearance_low_);
        getPrivateNodeHandle().param("clearance_high", proc_.clearance_high_, proc_.clearance_high_);
        getPrivateNodeHandle().param("obstacle_weight", proc_.obstacle_weight_, proc_.obstacle_weight_);
        getPrivateNodeHandle().param("remove_low_support", proc_.remove_low_support_, proc_.remove_low_support_);
        getPrivateNodeHandle().param("fixed_frame", fixed_frame_, fixed_frame_);
        getPrivateNodeHandle().param("timeout", timeout_, timeout_);
        NODELET_INFO("Support radius: %.3g m", proc_.support_radius_);
        NODELET_INFO("Min support: %i", proc_.min_support_);
        NODELET_INFO("Inclination radius: %.3g m", proc_.inclination_radius_);
        NODELET_INFO("Inclination weight: %.3g", proc_.inclination_weight_);
        NODELET_INFO("Normal std weight: %.3g", proc_.normal_std_weight_);
        NODELET_INFO("Clearance radius: %.3g m", proc_.clearance_radius_);
        NODELET_INFO("Clearance low: %.3g m", proc_.clearance_low_);
        NODELET_INFO("Clearance high: %.3g m", proc_.clearance_high_);
        NODELET_INFO("Obstacle weight: %.3g", proc_.obstacle_weight_);
        NODELET_INFO("Remove points with low support: %i", proc_.remove_low_support_);
        NODELET_INFO("Fixed frame: %s", fixed_frame_.c_str());
        NODELET_INFO("Timeout: %.3g s", timeout_);
    }
    void advertise()
    {
        cloud_pub_ = getNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 2);
    }
    void subscribe()
    {
        if (!fixed_frame_.empty())
        {
            tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_);
        }
        cloud_sub_ = getNodeHandle().subscribe("input", 2, &TraversabilityNodelet::onCloud, this);
    }
    void onInit() override
    {
        updateParams();
        advertise();
        subscribe();
    }
    void onCloud(const sensor_msgs::PointCloud2::ConstPtr & msg)
    {
        Timer t;
        geometry_msgs::TransformStamped tf;
        tf.transform.rotation.w = 1.0;
        if (!fixed_frame_.empty())
        {
            try
            {
                tf = tf_.lookupTransform(fixed_frame_,
                                         msg->header.frame_id,
                                         msg->header.stamp,
                                         ros::Duration(timeout_));
            }
            catch (tf2::TransformException & ex)
            {
                NODELET_ERROR("Could not transform %s to %s: %s.",
                              msg->header.frame_id.c_str(),
                              fixed_frame_.c_str(),
                              ex.what());
                return;
            }
            NODELET_INFO("Waited for transform: %f s.", t.seconds_elapsed());
        }
        t.reset();
        auto output = boost::make_shared<sensor_msgs::PointCloud2>();
        proc_.process(*msg, tf.transform, *output);
        cloud_pub_.publish(output);
        NODELET_INFO("Traversability estimated at %lu points: %f s.", num_points(*output), t.seconds_elapsed());
    }
};

}

PLUGINLIB_EXPORT_CLASS(naex::TraversabilityNodelet, nodelet::Nodelet

);
