#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <cmath>
#include <random>
#include <memory>
#include <optional>
#include <algorithm>

class LaserSimulator : public rclcpp::Node
{
public:
    LaserSimulator() : Node("laser_simulator")
    {
        // 初始化参数
        this->declare_parameter<double>("court_length", 15.0);
        this->declare_parameter<double>("court_width", 7.0);
        this->declare_parameter<int>("laser_count", 5);
        this->declare_parameter<double>("update_rate", 10.0);
        
        // 获取参数值
        court_length_ = this->get_parameter("court_length").as_double();
        court_width_ = this->get_parameter("court_width").as_double();
        laser_count_ = this->get_parameter("laser_count").as_int();
        update_rate_ = this->get_parameter("update_rate").as_double();
        
        // 初始化机器人状态
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> x_dist(1.0, court_length_-1);
        std::uniform_real_distribution<> y_dist(1.0, court_width_-1);
        std::uniform_real_distribution<> theta_dist(-M_PI/4, M_PI/4);
        
        // 设置初始位置（map坐标系下的绝对位置）
        map_x_ = x_dist(gen);
        map_y_ = y_dist(gen);
        map_theta_ = theta_dist(gen);
        // map_theta_ = 0.0;
        
        // odom坐标系下的位置（初始时与map坐标系重合）
        odom_x_ = 0.0;
        odom_y_ = 0.0;
        odom_theta_ = 0.0;
        
        // 创建静态TF广播器（用于发布一次性的静态变换）
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        
        // 发布初始的map->base_link静态变换（只发布一次）
        publishInitialTransform();
        is_first_time_ = true;
        
        // 创建cmd_vel订阅
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&LaserSimulator::cmdVelCallback, this, std::placeholders::_1));
            
        // 创建激光发布
        laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan", rclcpp::QoS(10));
        fake_laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("laser_scan_fake", rclcpp::QoS(10));

        // 创建动态TF广播器（用于持续更新的变换）
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        
        // 创建定时器
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate_)),
            std::bind(&LaserSimulator::updateSimulation, this));
    }

private:
    void publishInitialTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        
        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "odom";
        
        transform.transform.translation.x = 0.;
        transform.transform.translation.y = 0.;
        transform.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        // 发布静态变换（只发布一次）
        static_tf_broadcaster_->sendTransform(transform);
 
        RCLCPP_INFO(this->get_logger(), "Initial static transform map->odom published");
        RCLCPP_INFO(this->get_logger(), "Initial position (map frame): x %.2f, y %.2f, theta %.2f", map_x_, map_y_, map_theta_);
    }
    
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        // 保存最新的速度命令
        current_cmd_vel_ = *msg;
    }
    
    void updateSimulation()
    {
        // 1. 根据当前速度指令更新机器人在 map 坐标系下的真实位姿
        if (current_cmd_vel_) {
            const double dt = 1.0 / update_rate_;
            const double v  = current_cmd_vel_->linear.x;   // 机器人前向速度 (m/s)
            const double w  = current_cmd_vel_->angular.z;  // 机器人角速度 (rad/s)

            /*
             * 先更新角度，再使用新的航向角更新位置。
             * map 坐标系下的位置直接使用积分得到，而不再叠加累积的 odom 误差，
             * 这样可以避免随着时间推移出现位置畸变的问题。
             */
            map_theta_ += w * dt;
            // 将角度归一化到 [-pi, pi]
            map_theta_ = std::atan2(std::sin(map_theta_), std::cos(map_theta_));

            // 积分更新位置
            map_x_ += v * std::cos(map_theta_) * dt;
            map_y_ += v * std::sin(map_theta_) * dt;

            // 同步更新 odom 坐标（以初始点为原点）
            odom_theta_ += w * dt;
            odom_theta_ = std::atan2(std::sin(odom_theta_), std::cos(odom_theta_));
            odom_x_ += v * std::cos(odom_theta_) * dt;
            odom_y_ += v * std::sin(odom_theta_) * dt;

            // 保证机器人始终位于球场范围内
            map_x_ = std::max(0.0, std::min(court_length_, map_x_));
            map_y_ = std::max(0.0, std::min(court_width_, map_y_));

            RCLCPP_DEBUG(this->get_logger(), "Map pose   : x %.2f, y %.2f, theta %.2f", map_x_, map_y_, map_theta_);
            RCLCPP_DEBUG(this->get_logger(), "Odom pose  : x %.2f, y %.2f, theta %.2f", odom_x_, odom_y_, odom_theta_);
        }

        // 2. 发布 odom->base_link 的 TF
        publishOdomTransform();
        // 2.5 发布 map->fake_base_link 的 TF
        publishFakeBaseLinkTransform();

        // 3. 生成并发布激光数据（基于 map 坐标系的真实位置）
        auto laser_msg = simulateLaser();

        // 4. 先发布 fake_base_link 帧的 ground-truth 激光到 laser_scan_fake
        // RCLCPP_DEBUG(this->get_logger(), "Publish fake scan frame: %s", laser_msg->header.frame_id.c_str());
        // RCLCPP_INFO(this->get_logger(), "Publish fake scan frame: %s", laser_msg->header.frame_id.c_str());
        fake_laser_pub_->publish(*laser_msg);  // header.frame_id 保持为 fake_base_link

        // 5. 复制一份数据发布到 laser_scan 话题，frame_id 改为 base_link
        auto base_scan_msg = *laser_msg;  // 拷贝
        base_scan_msg.header.frame_id = "base_link";
        // RCLCPP_DEBUG(this->get_logger(), "Publish base scan frame: %s", base_scan_msg.header.frame_id.c_str());
        // RCLCPP_INFO(this->get_logger(), "Publish base scan frame: %s", base_scan_msg.header.frame_id.c_str());
        laser_pub_->publish(base_scan_msg);

        // 话题映射：
        //   laser_scan (base_link)
        //   laser_scan_fake (fake_base_link)
    }
    
    void publishOdomTransform()
    {
        geometry_msgs::msg::TransformStamped transform;
        
        transform.header.stamp = this->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        
        transform.transform.translation.x = odom_x_;
        transform.transform.translation.y = odom_y_;
        transform.transform.translation.z = 0.0;
        
        tf2::Quaternion q;
        q.setRPY(0, 0, odom_theta_);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();
        
        tf_broadcaster_->sendTransform(transform);
        
        
    }
    
    void publishFakeBaseLinkTransform()
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = "fake_base_link";

        transform.transform.translation.x = map_x_;
        transform.transform.translation.y = map_y_;
        transform.transform.translation.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, map_theta_);
        transform.transform.rotation.x = q.x();
        transform.transform.rotation.y = q.y();
        transform.transform.rotation.z = q.z();
        transform.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(transform);
    }
    
    std::shared_ptr<sensor_msgs::msg::LaserScan> simulateLaser()
    {
        auto laser_msg = std::make_shared<sensor_msgs::msg::LaserScan>();
        
        laser_msg->header.stamp = this->now();
        laser_msg->header.frame_id = "fake_base_link";
        
        // 激光参数设置
        laser_msg->angle_min = 0.0;
        laser_msg->angle_max = 2 * M_PI;
        laser_msg->angle_increment = 2 * M_PI / laser_count_;
        laser_msg->time_increment = 0.0;
        laser_msg->scan_time = 1.0 / update_rate_;
        laser_msg->range_min = 0.1;
        laser_msg->range_max = 20.0;
        
        // 计算激光束的距离（基于map坐标系）
        for (int i = 0; i < laser_count_; ++i) {
            // 计算激光在map坐标系中的绝对角度
            double absolute_angle = map_theta_ + i * 2 * M_PI / laser_count_;
            
            // 计算激光与边界的交点
            double distance = calculateDistanceToBoundary(absolute_angle);
            
            laser_msg->ranges.push_back(distance);
            if(is_first_time_){
                RCLCPP_INFO(this->get_logger(), "Laser %d: distance = %.2f", i, distance);
            }
        }
        is_first_time_ = false;
        
        return laser_msg;
    }
    
    double calculateDistanceToBoundary(double angle)
    {
        // 计算从当前位置沿给定角度到边界的距离
        
        // 处理角度在0-2π范围内
        angle = fmod(angle, 2 * M_PI);
        if (angle < 0) angle += 2 * M_PI;
        
        // 计算与四条边界的交点
        double distances[4];
        
        // 1. 与x=0的交点
        if (cos(angle) < -0.001) { // 向左
            double t = -map_x_ / cos(angle);
            double y_intersect = map_y_ + t * sin(angle);
            if (y_intersect >= 0 && y_intersect <= court_width_) {
                distances[0] = t;
            } else {
                distances[0] = std::numeric_limits<double>::infinity();
            }
        } else {
            distances[0] = std::numeric_limits<double>::infinity();
        }
        
        // 2. 与x=court_length_的交点
        if (cos(angle) > 0.001) { // 向右
            double t = (court_length_ - map_x_) / cos(angle);
            double y_intersect = map_y_ + t * sin(angle);
            if (y_intersect >= 0 && y_intersect <= court_width_) {
                distances[1] = t;
            } else {
                distances[1] = std::numeric_limits<double>::infinity();
            }
        } else {
            distances[1] = std::numeric_limits<double>::infinity();
        }
        
        // 3. 与y=0的交点
        if (sin(angle) < -0.001) { // 向下
            double t = -map_y_ / sin(angle);
            double x_intersect = map_x_ + t * cos(angle);
            if (x_intersect >= 0 && x_intersect <= court_length_) {
                distances[2] = t;
            } else {
                distances[2] = std::numeric_limits<double>::infinity();
            }
        } else {
            distances[2] = std::numeric_limits<double>::infinity();
        }
        
        // 4. 与y=court_width_的交点
        if (sin(angle) > 0.001) { // 向上
            double t = (court_width_ - map_y_) / sin(angle);
            double x_intersect = map_x_ + t * cos(angle);
            if (x_intersect >= 0 && x_intersect <= court_length_) {
                distances[3] = t;
            } else {
                distances[3] = std::numeric_limits<double>::infinity();
            }
        } else {
            distances[3] = std::numeric_limits<double>::infinity();
        }
        
        // 返回最小的有效距离
        double min_distance = *std::min_element(distances, distances + 4);
        return min_distance;
    }
    
    // 机器人状态（map坐标系下的绝对位置）
    double map_x_, map_y_, map_theta_;
    
    // 机器人状态（odom坐标系下的相对位置）
    double odom_x_, odom_y_, odom_theta_;
    
    // 篮球场尺寸
    double court_length_, court_width_;
    
    // 激光参数
    int laser_count_;
    double update_rate_;
    
    // ROS2相关
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr fake_laser_pub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 当前速度命令
    std::optional<geometry_msgs::msg::Twist> current_cmd_vel_;
    bool is_first_time_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LaserSimulator>());
    rclcpp::shutdown();
    return 0;
}