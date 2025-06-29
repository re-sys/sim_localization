#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>

class ScanListener : public rclcpp::Node
{
public:
  ScanListener() : Node("scan_listener"), map_to_odom_sent_(false)
  {
    // 参数：订阅激光话题名
    this->declare_parameter<std::string>("scan_topic", "laser_scan");
    std::string scan_topic = this->get_parameter("scan_topic").as_string();

    // 订阅 LaserScan
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic,
        rclcpp::SensorDataQoS(),
        std::bind(&ScanListener::scanCallback, this, std::placeholders::_1));

    // 订阅 RViz 发布的 initialpose
    init_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10,
        std::bind(&ScanListener::initialPoseCallback, this, std::placeholders::_1));

    // TF2 Buffer / Listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

    // Static broadcaster for map->odom
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "ScanListener started. Subscribed to scan topic: %s", scan_topic.c_str());
  }

private:
  // --- Callbacks -----------------------------------------------------------
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    if (!msg->ranges.empty())
    {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "[frame: %s] first range: %.2f m (array size: %zu)",
                           msg->header.frame_id.c_str(), msg->ranges[0], msg->ranges.size());
    }
  }

  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    // if (map_to_odom_sent_)
    // {
    //   RCLCPP_WARN(this->get_logger(), "map->odom static transform already sent. Ignoring subsequent initialpose.");
    //   return;
    // }

    // 获取 odom -> base_link 的实时变换
    geometry_msgs::msg::TransformStamped odom_to_base;
    try
    {
      odom_to_base = tf_buffer_->lookupTransform("odom", "base_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to lookup transform odom->base_link: %s", ex.what());
      return;
    }

    // 将 initial pose (map->base_link) 转为 tf2::Transform
    tf2::Transform tf_map_to_base;
    tf2::fromMsg(msg->pose.pose, tf_map_to_base);

    // 将 odom->base_link 转为 tf2::Transform
    tf2::Transform tf_odom_to_base;
    tf2::fromMsg(odom_to_base.transform, tf_odom_to_base);

    // 计算 map->odom: map->odom = map->base * (odom->base)^-1
    tf2::Transform tf_map_to_odom = tf_map_to_base * tf_odom_to_base.inverse();

    // 发布静态 TF
    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = msg->header.stamp;
    map_to_odom_msg.header.frame_id = "map";
    map_to_odom_msg.child_frame_id = "odom";
    map_to_odom_msg.transform = tf2::toMsg(tf_map_to_odom);

    static_broadcaster_->sendTransform(map_to_odom_msg);
    map_to_odom_sent_ = true;

    RCLCPP_INFO(this->get_logger(), "Published static transform map->odom based on initial pose.");
  }

  // --- Members -------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

  bool map_to_odom_sent_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanListener>());
  rclcpp::shutdown();
  return 0;
} 