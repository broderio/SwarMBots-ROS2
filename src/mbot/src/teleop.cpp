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

Teleop::Teleop()
: Node("teleop"), curr_bot(0)
{
    // Create publishers
    std::vector<std::string> mbot_topics = get_mbot_topics("/robot_vel_cmd");
    num_bots = mbot_topics.size();
    for (const auto & topic : mbot_topics) {
        RCLCPP_INFO(this->get_logger(), "Creating publisher for %s", topic.c_str());
        robot_vel_pubs.push_back(this->create_publisher<geometry_msgs::msg::TwistStamped>(topic, 10));
    }

    // Start teleop thread
    teleop_th_handle = std::thread(&Teleop::teleop, this);
}

std::vector<std::string> Teleop::get_mbot_topics(std::string topic_name)
{
    std::vector<std::string> mbot_topics;
    auto topics = this->get_topic_names_and_types();
    for (const auto & topic : topics) {
        if (topic.first.find("/mbot") == 0 && topic.first.find(topic_name) != std::string::npos) {
            mbot_topics.push_back(topic.first);
        }
    }
    return mbot_topics;
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

        // Change robot
        if (key <= '8' && key >= '1')
        {
            publish_vel(0.0, 0.0);
            int new_bot = key - '1';
            curr_bot = (new_bot < num_bots) ? new_bot : curr_bot;
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
    msg.twist.linear.x = vx;
    msg.twist.linear.y = 0.0;
    msg.twist.angular.z = wz;

    // Publish message
    robot_vel_pubs[curr_bot]->publish(msg);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);;
    rclcpp::spin(std::make_shared<Teleop>());
    rclcpp::shutdown();
}