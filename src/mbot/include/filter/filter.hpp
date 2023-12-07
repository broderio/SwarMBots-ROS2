#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"

// ROS2 message types
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "mbot_interfaces/msg/encoders.hpp"
#include "mbot_interfaces/msg/pwm.hpp"
#include "mbot_interfaces/msg/motors_vel.hpp"

using namespace std::chrono_literals;

class Filter : public rclcpp::Node 
{
public:
    Filter(const std::string & mac_address);
private:
    std::string mac_address;
    uint64_t max_msg_offset_ms = 60;
    uint64_t prev_utime = 0;
    double prev_yaw = 0.0;
    double alpha = 0.98;

    std::mutex msg_mutex;

    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr robot_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_pose_pub;

    geometry_msgs::msg::TwistStamped::UniquePtr latest_robot_vel;
    sensor_msgs::msg::Imu::UniquePtr latest_imu;
    geometry_msgs::msg::PoseStamped::UniquePtr latest_filtered_pose;

    void robot_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void process_messages();
};