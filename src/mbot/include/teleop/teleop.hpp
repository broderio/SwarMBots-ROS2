#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>

#include <curses.h>

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

class Teleop : public rclcpp::Node 
{
public:
    Teleop(const std::string & mac_address);
private:
    void teleop();
    void publish_vel(const float & vx, const float & wz);

    std::string mac_address;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr robot_vel_pub;
    std::thread teleop_th_handle;
};