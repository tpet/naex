#include <naex/traversability.h>
#include <naex/timer.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace naex
{

class TraversabilityNodelet : public nodelet::Nodelet
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
    getPrivateNodeHandle().param("radius", proc_.radius_, proc_.radius_);
    getPrivateNodeHandle().param("max_dist", proc_.max_dist_, proc_.max_dist_);
    getPrivateNodeHandle().param("min_support", proc_.min_support_, proc_.min_support_);
    getPrivateNodeHandle().param("inclination_weight", proc_.inclination_weight_, proc_.inclination_weight_);
    getPrivateNodeHandle().param("normal_std_weight", proc_.normal_std_weight_, proc_.normal_std_weight_);
    getPrivateNodeHandle().param("fixed_frame", fixed_frame_, fixed_frame_);
    getPrivateNodeHandle().param("timeout", timeout_, timeout_);
    NODELET_INFO("Radius: %.3f m", proc_.radius_);
    NODELET_INFO("Max dist: %.3f m", proc_.max_dist_);
    NODELET_INFO("Min support: %i", proc_.min_support_);
    NODELET_INFO("Inclination weight: %.3f", proc_.inclination_weight_);
    NODELET_INFO("Normal std. weight: %.3f", proc_.normal_std_weight_);
    NODELET_INFO("Fixed frame: %s", fixed_frame_.c_str());
    NODELET_INFO("Timeout: %.3f s", timeout_);
  }
  void advertise()
  {
    cloud_pub_ = getNodeHandle().advertise<sensor_msgs::PointCloud2>("output", 2);
  }
  void subscribe()
  {
    if (!fixed_frame_.empty())
      tf_listener_ = std::make_unique<tf2_ros::TransformListener>(tf_);
    cloud_sub_ = getNodeHandle().subscribe("input", 2, &TraversabilityNodelet::onCloud, this);
  }
  void onInit() override
  {
    updateParams();
    advertise();
    subscribe();
  }
  void onCloud(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    Timer t;
    geometry_msgs::TransformStamped tf;
    tf.transform.rotation.w = 1.0;
    if (!fixed_frame_.empty())
    {
      try
      {
        tf = tf_.lookupTransform(fixed_frame_, msg->header.frame_id, msg->header.stamp, ros::Duration(timeout_));
      }
      catch (tf2::TransformException& ex)
      {
        NODELET_ERROR("Could not transform %s to %s: %s.", msg->header.frame_id.c_str(), fixed_frame_.c_str(), ex.what());
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

PLUGINLIB_EXPORT_CLASS(naex::TraversabilityNodelet, nodelet::Nodelet);
