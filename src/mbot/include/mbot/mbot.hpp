#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <queue>
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

using namespace std::chrono_literals;
using boost::asio::ip::tcp;

class MbotMain : public rclcpp::Node
{
  public:
    MbotMain();
    ~MbotMain();

  private:
    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub;
    rclcpp::Publisher<mbot_interfaces::msg::MotorsVel>::SharedPtr motor_vel_pub;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr robot_vel_pub;
    rclcpp::Publisher<mbot_interfaces::msg::Pwm>::SharedPtr motor_pwm_pub;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr odom_pub;
    rclcpp::Publisher<mbot_interfaces::msg::Encoders>::SharedPtr enc_pub;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr timesync_sub;
    rclcpp::Subscription<mbot_interfaces::msg::MotorsVel>::SharedPtr motor_vel_sub;
    rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr robot_vel_sub;
    rclcpp::Subscription<mbot_interfaces::msg::Pwm>::SharedPtr motor_pwm_sub;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr odom_sub;
    rclcpp::Subscription<mbot_interfaces::msg::Encoders>::SharedPtr enc_sub;

    // Callbacks
    void timesync_callback(const std_msgs::msg::Header::SharedPtr msg);
    void motor_vel_callback(const mbot_interfaces::msg::MotorsVel::SharedPtr msg);
    void robot_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg);
    void motor_pwm_callback(const mbot_interfaces::msg::Pwm::SharedPtr msg);
    void odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void enc_callback(const mbot_interfaces::msg::Encoders::SharedPtr msg);

    // UART Handler Thread Functions & Variables
    void recv_th();
    boost::asio::io_service io_service_;
    tcp::socket socket_;
    std::thread recv_th_handle;
    std::mutex socket_mutex;
    std::string current_mac;

    // Message Wrapper Struct
    struct __attribute__((__packed__)) packets_wrapper_t
    {
        serial_mbot_encoders_t encoders;
        serial_pose2D_t odom;
        serial_mbot_imu_t imu;
        serial_twist2D_t mbot_vel;
        serial_mbot_motor_vel_t motor_vel;
        serial_mbot_motor_pwm_t motor_pwm;
    };

    // Publisher Helper Functions
    void publish_encoders(const serial_mbot_encoders_t & encoders) const;
    void publish_odom(const serial_pose2D_t & odom) const;
    void publish_imu(const serial_mbot_imu_t & imu) const;
    void publish_mbot_vel(const serial_twist2D_t & mbot_vel) const;
    void publish_motor_vel(const serial_mbot_motor_vel_t & motor_vel) const;
    void publish_motor_pwm(const serial_mbot_motor_pwm_t & motor_pwm) const;

    // Serial helper Functions

    std::string mac_bytes_to_string(const uint8_t mac_address[6]) const;

    void mac_string_to_bytes(const std::string & mac_str, 
                                   uint8_t       mac_address[6]) const;

    uint8_t checksum(const uint8_t * const addends, 
                     const int     &       len) const;
                
    void encode_msg(const uint8_t  * const in_msg, 
                    const uint16_t &       in_msg_len,
                    const uint16_t &       topic, 
                    const uint8_t          mac_address[6],
                          uint8_t  *       out_pkt) const;
};