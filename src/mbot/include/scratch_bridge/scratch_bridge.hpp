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

class ScratchBridge : public rclcpp::Node
{
  public:
    ScratchBridge();

  private:

    uint64_t max_msg_offset_ms = 60;
    uint64_t prev_utime = 0;
    double prev_yaw = 0.0;
    double alpha = 0.98;

    net::io_context ioc_;
    tcp::acceptor acceptor_;
    websocket::stream<tcp::socket> ws_;

    void socket_th();
    std::thread socket_th_handle;
    std::mutex socket_mutex;

    std::mutex msg_mutex;
    geometry_msgs::msg::TwistStamped latest_robot_vel;
    sensor_msgs::msg::Imu latest_imu;

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr robot_vel_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub;

    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr robot_vel_pub;

    // Callbacks
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void robot_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);

    void process_messages();

    // Message Wrapper Structs
    struct ScratchDriveAtCmd
    {
        float lin; // m/s
        float ang; // rad
    };
    void drive_at(const ScratchDriveAtCmd & cmd);

    struct ScratchDriveToCmd
    {
        bool type; // 0: linear, 1: angular
        float dist; // m or rad
    };
    ScratchDriveToCmd drive_to_cmd;
    float accumulated_err = 0.0;
    float Kp = 0.5;
    float Ki = 0.05;
    void drive_to(const ScratchDriveToCmd & cmd);

    struct ScratchMbotMsg
    {
      float x;  // m
      float y;  // m
      float heading;  // rad
      bool atGoal;
      float nearestObstacleDist;  // m
      float nearestObstacleAngle; // rad
    };
    ScratchMbotMsg mbot_msg;
};