#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <curses.h>

#include "teleop/teleop.hpp"

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

Teleop::Teleop(const std::string & mac_address)
: Node("teleop")
{
    // Create publisher
    robot_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mbot/robot_vel_cmd", 10);

    // Start teleop thread
    teleop_th_handle = std::thread(&Teleop::teleop, this);

    this->mac_address = mac_address;
}

void Teleop::teleop()
{
    // Initialize curses
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);

    // Initialize variables
    float vx = 0.0;
    float wz = 0.0;
    char key;

    while (rclcpp::ok())
    {
        // Get key pressed
        key = getch();

        vx = 0.0;
        wz = 0.0;
        if (key == 'q')
        {
            publish_vel(0.0, 0.0);
            break;
        }

        // Update vx and wz
        switch (key)
        {
            case 'w':
                vx = 0.2;
                break;
            case 's':
                vx = -0.2;
                break;
            case 'a':
                wz = 1.5;
                break;
            case 'd':
                wz = -1.5;
                break;
            default:
                break;
        }

        // Publish velocity
        publish_vel(vx, wz);
        usleep(100000);
    }

    // End curses
    endwin();
    exit(0);
}

void Teleop::publish_vel(const float & vx, const float & wz)
{
    // Create message
    auto msg = geometry_msgs::msg::TwistStamped();
    msg.header.stamp = this->now();
    msg.header.frame_id = mac_address;
    msg.twist.linear.x = vx;
    msg.twist.linear.y = 0.0;
    msg.twist.angular.z = wz;

    // Publish message
    robot_vel_pub->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    std::string mac_address = "30:30:f9:69:4e:49";
    rclcpp::spin(std::make_shared<Teleop>(mac_address));
    rclcpp::shutdown();
}