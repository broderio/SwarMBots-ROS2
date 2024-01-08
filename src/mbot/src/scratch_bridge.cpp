#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
#include <unordered_map>
#include <thread>
#include <mutex>
#include <iostream>
#include <cstring>
#include <type_traits>
#include <sstream>

#include <boost/asio.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "scratch_bridge/scratch_bridge.hpp"

#include "comms.h"
#include "serial_msg.h"

#include "rclcpp/rclcpp.hpp"

// ROS2 message types
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/string.hpp"

#include "sensor_msgs/msg/imu.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "mbot_interfaces/msg/encoders.hpp"
#include "mbot_interfaces/msg/pwm.hpp"
#include "mbot_interfaces/msg/motors_vel.hpp"

#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/asio/ip/tcp.hpp>

namespace beast = boost::beast; // from <boost/beast.hpp>
namespace websocket = beast::websocket; // from <boost/beast/websocket.hpp>
namespace net = boost::asio; // from <boost/asio.hpp>
using tcp = boost::asio::ip::tcp; // from <boost/asio/ip/tcp.hpp>

using namespace std::chrono_literals;
using boost::asio::ip::tcp;

ScratchBridge::ScratchBridge() 
: Node("scratch_bridge"), acceptor_(ioc_, tcp::endpoint(tcp::v4(), 8765)), ws_(ioc_)
{
    RCLCPP_INFO(this->get_logger(), "ScratchBridge node started");

    // Accept a connection
    acceptor_.accept(ws_.next_layer());

    RCLCPP_INFO(this->get_logger(), "ScratchBridge socket accepted");

    // Perform the websocket handshake
    ws_.accept();

    RCLCPP_INFO(this->get_logger(), "ScratchBridge socket handshake accepted");

    mbot_msg = {
        .x = 0.0,
        .y = 0.0,
        .heading = 0.0,
        .atGoal = true,
        .nearestObstacleDist = 0.0,
        .nearestObstacleAngle = 0.0,
    };

    // Create publisher
    robot_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mbot0/robot_vel_cmd", 10);

    // Create subscribers
    robot_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>("/mbot0/robot_vel", 10, std::bind(&ScratchBridge::robot_vel_callback, this, std::placeholders::_1));
    imu_sub = this->create_subscription<sensor_msgs::msg::Imu>("/mbot0/imu", 10, std::bind(&ScratchBridge::imu_callback, this, std::placeholders::_1));

    // Start socket_th
    socket_th_handle = std::thread(&ScratchBridge::socket_th, this);
}

void ScratchBridge::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(msg_mutex);
        latest_imu = *std::move(msg);
    }
    process_messages();
}

void ScratchBridge::robot_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    {
        std::lock_guard<std::mutex> lock(msg_mutex);
        latest_robot_vel = *std::move(msg);
    }
    process_messages();
}

void ScratchBridge::process_messages()
{
    std::lock_guard<std::mutex> lock(msg_mutex);

    if (prev_utime == 0) {
        prev_utime = latest_robot_vel.header.stamp.sec * 1000 + latest_robot_vel.header.stamp.nanosec / 1000000;
        return; 
    }

    uint64_t imu_time_ms = latest_imu.header.stamp.sec * 1000 + latest_imu.header.stamp.nanosec / 1000000;
    uint64_t robot_vel_time_ms = latest_robot_vel.header.stamp.sec * 1000 + latest_robot_vel.header.stamp.nanosec / 1000000;
    uint64_t time_diff_ms = imu_time_ms > robot_vel_time_ms ? imu_time_ms - robot_vel_time_ms : robot_vel_time_ms - imu_time_ms;
    if (time_diff_ms > this->max_msg_offset_ms) return;

    // Calculate dt in seconds
    double dt = (robot_vel_time_ms - prev_utime) / 1000.0;

    // Convert imu quaternion to rpy
    tf2::Quaternion q;
    tf2::fromMsg(latest_imu.orientation, q);
    double r, p, yaw_imu;
    tf2::Matrix3x3(q).getRPY(r, p, yaw_imu);

    // Calculate dyaw
    double dyaw_imu = yaw_imu - prev_yaw;
    double dyaw_odom = latest_robot_vel.twist.angular.z * dt;
    double dyaw_filtered = dyaw_imu * alpha + dyaw_odom * (1 - alpha);
    mbot_msg.heading += dyaw_filtered;

    // Calculate new pose position
    double dx_space = (latest_robot_vel.twist.linear.x * cos(mbot_msg.heading) - latest_robot_vel.twist.linear.y * sin(mbot_msg.heading)) * dt;
    double dy_space = (latest_robot_vel.twist.linear.x * sin(mbot_msg.heading) + latest_robot_vel.twist.linear.y * cos(mbot_msg.heading)) * dt;
    mbot_msg.x += dx_space;
    mbot_msg.y += dy_space;

    // Update prev_utime and prev_yaw
    prev_utime = robot_vel_time_ms;
    prev_yaw = yaw_imu;

    // Check if there is a goal to drive to
    if (!mbot_msg.atGoal)
    {
        // Set distance to goal
        float err = drive_to_cmd.dist;
        accumulated_err += err;

        // Check if goal is reached
        if (err < 0.02)
        {
            mbot_msg.atGoal = true;
            accumulated_err = 0.0;
            drive_at({0.0, 0.0});
            return;
        }

        // Calculate new velocity
        if (drive_to_cmd.type == 0)
        {
            // Linear
            float lin = err * Kp + accumulated_err * Ki;
            drive_at({lin, 0.0});
        }
        else
        {
            // Angular
            float ang = err * Kp + accumulated_err * Ki;
            drive_at({0.0, ang});
        }
    }

    // Publish message to socket
    std::stringstream ss;
    ss << "MB," << mbot_msg.x << "," << mbot_msg.y << "," << mbot_msg.heading << "," << mbot_msg.atGoal << "\n";
    std::string msg = ss.str();
    RCLCPP_INFO(this->get_logger(), "Sending message: %s", msg.c_str());
    ws_.write(net::buffer(msg));
}

// Take Scratch Drive At command and publish to /mbot0/robot_vel_cmd
void ScratchBridge::drive_at(const ScratchDriveAtCmd &cmd)
{
    geometry_msgs::msg::TwistStamped robot_vel_cmd;
    robot_vel_cmd.header.stamp = this->now();
    robot_vel_cmd.twist.linear.x = cmd.lin;
    robot_vel_cmd.twist.angular.z = cmd.ang;
    robot_vel_pub->publish(robot_vel_cmd);
}

// Take Scratch Drive To command and set new goal
void ScratchBridge::drive_to(const ScratchDriveToCmd &cmd)
{
    this->drive_to_cmd = cmd;
    mbot_msg.atGoal = false;
}

void ScratchBridge::socket_th()
{
    beast::multi_buffer buffer;
    std::string flag;
    while (rclcpp::ok())
    {
        // Read a message into our buffer
        ws_.read(buffer);

        // The make_printable() function helps print a ConstBufferSequence
        std::string message = boost::beast::buffers_to_string(buffer.data());

        std::istringstream is(message);
        std::getline(is, flag, ',');
        
        if (flag == "DA") {
            ScratchDriveAtCmd cmd;
            std::string lin, ang;
            std::getline(is, lin, ',');
            std::getline(is, ang);
            RCLCPP_INFO(this->get_logger(), "Received DriveAt cmd: lin: %s, ang: %s", lin.c_str(), ang.c_str());
            cmd.lin = std::stof(lin);
            cmd.ang = std::stof(ang);
            drive_at(cmd);
        } 
        else if (flag == "DT") {
            ScratchDriveToCmd cmd;
            std::string type, dist;
            std::getline(is, type, ',');
            std::getline(is, dist);
            cmd.type = type == "1";
            cmd.dist = std::stof(dist);
            drive_to(cmd);
        }
        buffer.consume(buffer.size());
    }
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScratchBridge>());
    rclcpp::shutdown();
    return 0;
}