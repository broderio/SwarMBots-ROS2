#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "filter/filter.hpp"

#include "rclcpp/rclcpp.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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

Filter::Filter(const std::string & mac_address)
: Node("comp_filter")
{
    // Create publisher
    filtered_pose_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mbot/filtered_pose", 10);

    // Create subscribers
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/mbot/imu", 10, std::bind(&Filter::imu_callback, this, std::placeholders::_1));
    robot_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>("/mbot/robot_vel", 10, std::bind(&Filter::robot_vel_callback, this, std::placeholders::_1));

    this->mac_address = mac_address;
    latest_filtered_pose->header.frame_id = mac_address;
}

void Filter::robot_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(msg_mutex);
        *latest_robot_vel = *std::move(msg);
    }
    process_messages();
}

void Filter::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(msg_mutex);
        *latest_imu = *std::move(msg);
    }
    process_messages();
}

void Filter::process_messages()
{
    std::lock_guard<std::mutex> lock(msg_mutex);

    if (!latest_robot_vel || !latest_imu) return;

    if (prev_utime == 0) {
        prev_utime = latest_robot_vel->header.stamp.sec * 1000 + latest_robot_vel->header.stamp.nanosec / 1000000;
        return; 
    }

    uint64_t imu_time_ms = latest_imu->header.stamp.sec * 1000 + latest_imu->header.stamp.nanosec / 1000000;
    uint64_t robot_vel_time_ms = latest_robot_vel->header.stamp.sec * 1000 + latest_robot_vel->header.stamp.nanosec / 1000000;
    uint64_t time_diff_ms = imu_time_ms > robot_vel_time_ms ? imu_time_ms - robot_vel_time_ms : robot_vel_time_ms - imu_time_ms;
    if (time_diff_ms > max_msg_offset_ms) return;

    // Calculate dt in seconds
    double dt = (robot_vel_time_ms - prev_utime) / 1000.0;

    // Convert imu quaternion to rpy
    tf2::Quaternion q;
    tf2::fromMsg(latest_imu->orientation, q);
    double r, p, yaw_imu;
    tf2::Matrix3x3(q).getRPY(r, p, yaw_imu);

    // Calculate dyaw from IMU
    double dyaw_imu = yaw_imu - prev_yaw;

    // Calculate dx, dy, dyaw from velocity
    double dx_space = (latest_robot_vel->twist.linear.x * cos(yaw_imu) - latest_robot_vel->twist.linear.y * sin(yaw_imu)) * dt;
    double dy_space = (latest_robot_vel->twist.linear.x * sin(yaw_imu) + latest_robot_vel->twist.linear.y * cos(yaw_imu)) * dt;
    double dyaw_odom = latest_robot_vel->twist.angular.z * dt;

    // Calculate new yaw
    double dyaw_filtered = dyaw_imu * alpha + dyaw_odom * (1 - alpha);

    // Convert dyaw to quaternion
    tf2::Quaternion q_dyaw;
    q_dyaw.setRPY(0.0, 0.0, dyaw_filtered);

    // Calculate new pose quaternion
    tf2::Quaternion q_pose;
    tf2::fromMsg(latest_filtered_pose->pose.orientation, q_pose);
    tf2::Quaternion q_new = q_pose * q_dyaw;
    latest_filtered_pose->pose.orientation = tf2::toMsg(q_new);

    // Calculate new pose position
    latest_filtered_pose->pose.position.x += dx_space;
    latest_filtered_pose->pose.position.y += dy_space;

    // Update prev_utime and prev_yaw
    prev_utime = robot_vel_time_ms;
    prev_yaw = yaw_imu;

    // Publish filtered pose
    filtered_pose_pub->publish(*latest_filtered_pose);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string mac_address = "30:30:f9:69:4e:49";
    rclcpp::spin(std::make_shared<Filter>(mac_address));
    rclcpp::shutdown();
}