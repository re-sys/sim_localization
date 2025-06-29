#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <cmath>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Transform.h>

class SensorDataToScan : public rclcpp::Node
{
public:
  SensorDataToScan() : Node("sensor_data_to_scan")
  {
    // Declare parameters with default values
    this->declare_parameter<std::string>("sensor_topic", "sensor_data");
    this->declare_parameter<std::string>("scan_topic", "scan");
    this->declare_parameter<int>("sensor_count", 5);
    this->declare_parameter<double>("first_angle_deg", 60.0);
    this->declare_parameter<double>("angle_increment_deg", 72.0);
    this->declare_parameter<double>("range_min", 0.02);
    this->declare_parameter<double>("range_max", 20.0);

    // Read parameters
    auto sensor_topic = this->get_parameter("sensor_topic").as_string();
    auto scan_topic   = this->get_parameter("scan_topic").as_string();
    sensor_count_     = this->get_parameter("sensor_count").as_int();
    first_angle_      = this->get_parameter("first_angle_deg").as_double() * M_PI / 180.0;
    angle_increment_  = this->get_parameter("angle_increment_deg").as_double() * M_PI / 180.0;
    range_min_        = this->get_parameter("range_min").as_double();
    range_max_        = this->get_parameter("range_max").as_double();

    // Create subscriber and publisher
    sensor_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        sensor_topic, 10,
        std::bind(&SensorDataToScan::sensorCallback, this, std::placeholders::_1));

    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, 10);

    // TF utilities for initialpose handling
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);
    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    // Subscribe to RViz initialpose
    init_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "initialpose", 10,
        std::bind(&SensorDataToScan::initialPoseCallback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "SensorDataToScan node started. Subscribing to '%s', publishing LaserScan on '%s'", sensor_topic.c_str(), scan_topic.c_str());
  }

private:
  void sensorCallback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (static_cast<int>(msg->data.size()) < sensor_count_)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                           "Received Float32MultiArray with size %zu, expected at least %d", msg->data.size(), sensor_count_);
      return;
    }

    // Prepare LaserScan message
    sensor_msgs::msg::LaserScan scan_msg;
    scan_msg.header.stamp = this->now();
    scan_msg.header.frame_id = "base_link";  // The ranges are relative to base_link

    scan_msg.angle_min = first_angle_;
    scan_msg.angle_increment = angle_increment_;
    scan_msg.angle_max = first_angle_ + angle_increment_ * (sensor_count_ - 1);

    scan_msg.time_increment = 0.0;
    scan_msg.scan_time = 0.0;
    scan_msg.range_min = range_min_;
    scan_msg.range_max = range_max_;

    scan_msg.ranges.resize(sensor_count_);
    scan_msg.intensities.resize(sensor_count_);

    for (int i = 0; i < sensor_count_; ++i)
    {
      scan_msg.ranges[i] = static_cast<float>(msg->data[i]);
      scan_msg.intensities[i] = 0.0f;  // No intensity information
    }

    scan_pub_->publish(scan_msg);
  }

  // Handle RViz 2D Pose Estimate to reset map->odom
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
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

    tf2::Transform tf_map_to_base;
    tf2::fromMsg(msg->pose.pose, tf_map_to_base);

    tf2::Transform tf_odom_to_base;
    tf2::fromMsg(odom_to_base.transform, tf_odom_to_base);

    tf2::Transform tf_map_to_odom = tf_map_to_base * tf_odom_to_base.inverse();

    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = msg->header.stamp;
    map_to_odom_msg.header.frame_id = "map";
    map_to_odom_msg.child_frame_id = "odom";
    map_to_odom_msg.transform = tf2::toMsg(tf_map_to_odom);

    static_broadcaster_->sendTransform(map_to_odom_msg);

    RCLCPP_INFO(this->get_logger(), "Updated static transform map->odom from initialpose.");
  }

  // Parameters
  int sensor_count_;
  double first_angle_;
  double angle_increment_;
  double range_min_;
  double range_max_;

  // TF and subscriptions for initialpose
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_sub_;

  // ROS interfaces
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sensor_sub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorDataToScan>());
  rclcpp::shutdown();
  return 0;
} 