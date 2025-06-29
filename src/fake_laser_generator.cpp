#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <limits>
#include <algorithm>

class FakeLaserGenerator : public rclcpp::Node
{
public:
  FakeLaserGenerator() : Node("fake_laser_generator")
  {
    // Declare parameters
    this->declare_parameter<std::string>("sensor_topic", "sensor_data");

    // Get parameter values
    court_length_ = getNumericParamAsDouble("court_length", 15.0);
    court_width_  = getNumericParamAsDouble("court_width", 7.0);
    sensor_count_ = static_cast<int>(getNumericParamAsDouble("sensor_count", 5));
    first_angle_  = getNumericParamAsDouble("first_angle_deg", 60.0) * M_PI / 180.0;
    angle_increment_ = getNumericParamAsDouble("angle_increment_deg", 72.0) * M_PI / 180.0;
    update_rate_  = getNumericParamAsDouble("update_rate", 10.0);
    auto sensor_topic = this->get_parameter("sensor_topic").as_string();

    // Publisher
    sensor_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(sensor_topic, 10);

    // TF listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Timer for periodic publishing
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
      std::bind(&FakeLaserGenerator::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "FakeLaserGenerator started. Publishing '%s' at %.1f Hz", sensor_topic.c_str(), update_rate_);
  }

private:
  void timerCallback()
  {
    // Lookup map -> fake_base_link transform
    geometry_msgs::msg::TransformStamped transform;
    try
    {
      transform = tf_buffer_->lookupTransform("map", "fake_base_link", tf2::TimePointZero);
    }
    catch (const tf2::TransformException & ex)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "TF unavailable: %s", ex.what());
      return;
    }

    // Extract position
    double map_x = transform.transform.translation.x;
    double map_y = transform.transform.translation.y;

    // Extract yaw from quaternion
    tf2::Quaternion q(
      transform.transform.rotation.x,
      transform.transform.rotation.y,
      transform.transform.rotation.z,
      transform.transform.rotation.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

    // Compute ranges
    std_msgs::msg::Float32MultiArray sensor_msg;
    sensor_msg.data.reserve(sensor_count_);

    for (int i = 0; i < sensor_count_; ++i)
    {
      double absolute_angle = yaw + first_angle_ + i * angle_increment_;
      double distance = calculateDistanceToBoundary(map_x, map_y, absolute_angle);
      sensor_msg.data.push_back(static_cast<float>(distance));
    }

    sensor_pub_->publish(sensor_msg);
  }

  double calculateDistanceToBoundary(double x, double y, double angle)
  {
    // Normalize angle to [0, 2pi)
    angle = fmod(angle, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;

    double distances[4];

    // Left boundary (x = 0)
    if (cos(angle) < -0.001)
    {
      double t = -x / cos(angle);
      double y_intersect = y + t * sin(angle);
      distances[0] = (y_intersect >= 0 && y_intersect <= court_width_) ? t : std::numeric_limits<double>::infinity();
    }
    else
    {
      distances[0] = std::numeric_limits<double>::infinity();
    }

    // Right boundary (x = court_length_)
    if (cos(angle) > 0.001)
    {
      double t = (court_length_ - x) / cos(angle);
      double y_intersect = y + t * sin(angle);
      distances[1] = (y_intersect >= 0 && y_intersect <= court_width_) ? t : std::numeric_limits<double>::infinity();
    }
    else
    {
      distances[1] = std::numeric_limits<double>::infinity();
    }

    // Bottom boundary (y = 0)
    if (sin(angle) < -0.001)
    {
      double t = -y / sin(angle);
      double x_intersect = x + t * cos(angle);
      distances[2] = (x_intersect >= 0 && x_intersect <= court_length_) ? t : std::numeric_limits<double>::infinity();
    }
    else
    {
      distances[2] = std::numeric_limits<double>::infinity();
    }

    // Top boundary (y = court_width_)
    if (sin(angle) > 0.001)
    {
      double t = (court_width_ - y) / sin(angle);
      double x_intersect = x + t * cos(angle);
      distances[3] = (x_intersect >= 0 && x_intersect <= court_length_) ? t : std::numeric_limits<double>::infinity();
    }
    else
    {
      distances[3] = std::numeric_limits<double>::infinity();
    }

    return *std::min_element(distances, distances + 4);
  }

  // Helper to safely fetch numeric parameter (accept int or double)
  double getNumericParamAsDouble(const std::string & name, double default_val)
  {
    rclcpp::Parameter param;
    if (this->get_parameter(name, param))
    {
      if (param.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        return param.as_double();
      }
      else if (param.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
      {
        return static_cast<double>(param.as_int());
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Parameter '%s' has non numeric type, using default %.2f", name.c_str(), default_val);
      }
    }
    return default_val;
  }

  // Parameters
  double court_length_;
  double court_width_;
  int sensor_count_;
  double first_angle_;
  double angle_increment_;
  double update_rate_;

  // ROS interfaces
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr sensor_pub_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FakeLaserGenerator>());
  rclcpp::shutdown();
  return 0;
} 