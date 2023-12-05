#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <mutex>
#include <condition_variable>

#include "mbot/mbot.hpp"

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

MbotMain::MbotMain(const std::string &serial_port)
: Node("mbot_main")
{
    // Initialize serial port
    serial_init(serial_port);

    // Initialize publishers
    imu_pub = this->create_publisher<sensor_msgs::msg::Imu>("/mbot/imu", 10);
    motor_vel_pub = this->create_publisher<mbot_interfaces::msg::MotorsVel>("/mbot/motor_vel", 10);
    robot_vel_pub = this->create_publisher<geometry_msgs::msg::TwistStamped>("/mbot/robot_vel", 10);
    motor_pwm_pub = this->create_publisher<mbot_interfaces::msg::Pwm>("/mbot/pwm", 10);
    odom_pub = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mbot/odom", 10);
    enc_pub = this->create_publisher<mbot_interfaces::msg::Encoders>("/mbot/encoders", 10);

    // Initialize subscribers
    timesync_sub = this->create_subscription<std_msgs::msg::Header>("/mbot/timesync", 10, std::bind(&MbotMain::timesync_callback, this, std::placeholders::_1));
    motor_vel_sub = this->create_subscription<mbot_interfaces::msg::MotorsVel>("/mbot/motor_vel_cmd", 10, std::bind(&MbotMain::motor_vel_callback, this, std::placeholders::_1));
    robot_vel_sub = this->create_subscription<geometry_msgs::msg::TwistStamped>("/mbot/robot_vel_cmd", 10, std::bind(&MbotMain::robot_vel_callback, this, std::placeholders::_1));
    motor_pwm_sub = this->create_subscription<mbot_interfaces::msg::Pwm>("/mbot/pwm_cmd", 10, std::bind(&MbotMain::motor_pwm_callback, this, std::placeholders::_1));
    odom_sub = this->create_subscription<geometry_msgs::msg::PoseStamped>("/mbot/odom_cmd", 10, std::bind(&MbotMain::odom_callback, this, std::placeholders::_1));
    enc_sub = this->create_subscription<mbot_interfaces::msg::Encoders>("/mbot/encoders_cmd", 10, std::bind(&MbotMain::enc_callback, this, std::placeholders::_1));

    // Start UART Handler Thread
    recv_th_handle = std::thread(&MbotMain::recv_th, this);
}

MbotMain::~MbotMain()
{
    // Stop UART Handler Threads
    recv_th_handle.join();
}

void MbotMain::timesync_callback(const std_msgs::msg::Header::SharedPtr msg)
{
    serial_timestamp_t out_msg;
    out_msg.utime = msg->stamp.sec * 1e6 + msg->stamp.nanosec / 1e3;

    uint8_t mac_addr[6];
    mac_string_to_bytes(msg->frame_id, mac_addr);

    uint8_t out_pkt[sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3];
    encode_msg((uint8_t*)&out_msg, sizeof(out_msg), MBOT_TIMESYNC, mac_addr, out_pkt);
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        write(serial_port_fd, out_pkt, sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3);
    }
}

void MbotMain::motor_vel_callback(const mbot_interfaces::msg::MotorsVel::SharedPtr msg)
{
    serial_mbot_motor_vel_t out_msg;
    out_msg.utime = msg->header.stamp.sec * 1e6 + msg->header.stamp.nanosec / 1e3;
    std::memcpy(out_msg.velocity, msg->velocity.data(), sizeof(float) * 3);

    uint8_t mac_addr[6];
    mac_string_to_bytes(msg->header.frame_id, mac_addr);

    uint8_t out_pkt[sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3];
    encode_msg((uint8_t*)&out_msg, sizeof(out_msg), MBOT_MOTOR_VEL_CMD, mac_addr, out_pkt);
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        write(serial_port_fd, out_pkt, sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3);
    }
}

void MbotMain::robot_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    serial_twist2D_t out_msg;
    out_msg.utime = msg->header.stamp.sec * 1e6 + msg->header.stamp.nanosec / 1e3;
    out_msg.vx = msg->twist.linear.x;
    out_msg.vy = msg->twist.linear.y;
    out_msg.wz = msg->twist.angular.z;

    uint8_t mac_addr[6];
    mac_string_to_bytes(msg->header.frame_id, mac_addr);

    uint8_t out_pkt[sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3];
    encode_msg((uint8_t*)&out_msg, sizeof(out_msg), MBOT_VEL_CMD, mac_addr, out_pkt);
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        write(serial_port_fd, out_pkt, sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3);
    }
}

void MbotMain::motor_pwm_callback(const mbot_interfaces::msg::Pwm::SharedPtr msg)
{
    serial_mbot_motor_pwm_t out_msg;
    out_msg.utime = msg->header.stamp.sec * 1e6 + msg->header.stamp.nanosec / 1e3;
    std::memcpy(out_msg.pwm, msg->pwm.data(), sizeof(float) * 3);

    uint8_t mac_addr[6];
    mac_string_to_bytes(msg->header.frame_id, mac_addr);

    uint8_t out_pkt[sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3];
    encode_msg((uint8_t*)&out_msg, sizeof(out_msg), MBOT_MOTOR_PWM_CMD, mac_addr, out_pkt);
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        write(serial_port_fd, out_pkt, sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3);
    }
}

void MbotMain::odom_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    serial_pose2D_t out_msg;
    out_msg.utime = msg->header.stamp.sec * 1e6 + msg->header.stamp.nanosec / 1e3;
    out_msg.x = msg->pose.position.x;
    out_msg.y = msg->pose.position.y;
    out_msg.theta = msg->pose.orientation.z;

    uint8_t mac_addr[6];
    mac_string_to_bytes(msg->header.frame_id, mac_addr);

    uint8_t out_pkt[sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3];
    encode_msg((uint8_t*)&out_msg, sizeof(out_msg), MBOT_ODOMETRY_RESET, mac_addr, out_pkt);
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        write(serial_port_fd, out_pkt, sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3);
    }
}   

void MbotMain::enc_callback(const mbot_interfaces::msg::Encoders::SharedPtr msg)
{
    serial_mbot_encoders_t out_msg;
    out_msg.utime = msg->header.stamp.sec * 1e6 + msg->header.stamp.nanosec / 1e3;
    out_msg.delta_time = msg->delta_time;
    std::memcpy(out_msg.ticks, msg->ticks.data(), sizeof(int64_t) * 3);
    std::memcpy(out_msg.delta_ticks, msg->delta_ticks.data(), sizeof(int32_t) * 3);

    uint8_t mac_addr[6];
    mac_string_to_bytes(msg->header.frame_id, mac_addr);

    uint8_t out_pkt[sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3];
    encode_msg((uint8_t*)&out_msg, sizeof(out_msg), MBOT_ENCODERS_RESET, mac_addr, out_pkt);
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        write(serial_port_fd, out_pkt, sizeof(out_msg) + ROS_PKG_LEN + MAC_ADDR_LEN + 3);
    }
}

void MbotMain::recv_th()
{
    while (rclcpp::ok())
    {
        uint8_t trigger_val = 0x00;

        // Wait for the trigger byte
        while (trigger_val != 0xff)
        {
            read(serial_port_fd, &trigger_val, 1);
        }

        // Read MAC address and packet length
        uint8_t mac[6];
        uint16_t pkt_len;
        read_mac_address(mac, &pkt_len);
        if (pkt_len != 204)
            continue;

        // Read message and checksum
        uint8_t* msg_data_serialized = new uint8_t[pkt_len];
        uint8_t data_checksum = 0;
        read_message(msg_data_serialized, pkt_len, &data_checksum);

        // Validate message
        if (!validate_message(msg_data_serialized, pkt_len, data_checksum))
            continue;

        // Deserialize and publish messages
        packets_wrapper_t *packets = (packets_wrapper_t *)msg_data_serialized;
        current_mac = mac_bytes_to_string(mac);
        publish_encoders(packets->encoders);
        publish_odom(packets->odom);
        publish_imu(packets->imu);
        publish_mbot_vel(packets->mbot_vel);
        publish_motor_vel(packets->motor_vel);
        publish_motor_pwm(packets->motor_pwm);
        delete[] msg_data_serialized;
    }
}

void MbotMain::publish_encoders(const serial_mbot_encoders_t &encoders) const
{
    mbot_interfaces::msg::Encoders msg;
    msg.header.stamp.sec = encoders.utime / 1e6;
    msg.header.stamp.nanosec = (encoders.utime % (int64_t)1e6) * 1e3;
    msg.header.frame_id = current_mac;
    msg.delta_time = encoders.delta_time;
    std::memcpy(msg.ticks.data(), encoders.ticks, sizeof(int64_t) * 3);
    std::memcpy(msg.delta_ticks.data(), encoders.delta_ticks, sizeof(int32_t) * 3);
    enc_pub->publish(msg);
}

void MbotMain::publish_odom(const serial_pose2D_t &odom) const
{
    geometry_msgs::msg::PoseStamped msg;
    msg.header.stamp.sec = odom.utime / 1e6;
    msg.header.stamp.nanosec = (odom.utime % (int64_t)1e6) * 1e3;
    msg.header.frame_id = current_mac;
    msg.pose.position.x = odom.x;
    msg.pose.position.y = odom.y;
    msg.pose.orientation.z = odom.theta;
    odom_pub->publish(msg);
}

void MbotMain::publish_imu(const serial_mbot_imu_t &imu) const
{
    sensor_msgs::msg::Imu msg;
    msg.header.stamp.sec = imu.utime / 1e6;
    msg.header.stamp.nanosec = (imu.utime % (int64_t)1e6) * 1e3;
    msg.header.frame_id = current_mac;
    msg.linear_acceleration.x = imu.accel[0];
    msg.linear_acceleration.y = imu.accel[1];
    msg.linear_acceleration.z = imu.accel[2];
    msg.angular_velocity.x = imu.gyro[0];
    msg.angular_velocity.y = imu.gyro[1];
    msg.angular_velocity.z = imu.gyro[2];
    msg.orientation.x = imu.angles_quat[0];
    msg.orientation.y = imu.angles_quat[1];
    msg.orientation.z = imu.angles_quat[2];
    msg.orientation.w = imu.angles_quat[3];
    imu_pub->publish(msg);
}

void MbotMain::publish_mbot_vel(const serial_twist2D_t &mbot_vel) const
{
    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp.sec = mbot_vel.utime / 1e6;
    msg.header.stamp.nanosec = (mbot_vel.utime % (int64_t)1e6) * 1e3;
    msg.header.frame_id = current_mac;
    msg.twist.linear.x = mbot_vel.vx;
    msg.twist.linear.y = mbot_vel.vy;
    msg.twist.angular.z = mbot_vel.wz;
    robot_vel_pub->publish(msg);
}

void MbotMain::publish_motor_vel(const serial_mbot_motor_vel_t &motor_vel) const
{
    mbot_interfaces::msg::MotorsVel msg;
    msg.header.stamp.sec = motor_vel.utime / 1e6;
    msg.header.stamp.nanosec = (motor_vel.utime % (int64_t)1e6) * 1e3;
    msg.header.frame_id = current_mac;
    std::memcpy(msg.velocity.data(), motor_vel.velocity, sizeof(float) * 3);
    motor_vel_pub->publish(msg);
}

void MbotMain::publish_motor_pwm(const serial_mbot_motor_pwm_t &motor_pwm) const
{
    mbot_interfaces::msg::Pwm msg;
    msg.header.stamp.sec = motor_pwm.utime / 1e6;
    msg.header.stamp.nanosec = (motor_pwm.utime % (int64_t)1e6) * 1e3;
    msg.header.frame_id = current_mac;
    std::memcpy(msg.pwm.data(), motor_pwm.pwm, sizeof(float) * 3);
    motor_pwm_pub->publish(msg);
}

void MbotMain::serial_init(const std::string &serial_port)
{
    // open com port host is connected to
    struct termios tty;
    serial_port_fd = open(serial_port.c_str(), O_RDWR);
    if (serial_port_fd == -1)
    {
        perror("Error: Unable to open serial port.\n");
        return;
    }

    if (tcgetattr(serial_port_fd, &tty) != 0)
    {
        perror("Error: Unable to get serial port attributes.\n");
        return;
    }

    cfsetospeed(&tty, B921600); // Set output baud rate
    cfsetispeed(&tty, B921600); // Set input baud rate

    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Disable software flow control
    tty.c_cflag |= (CLOCAL | CREAD);        // Ignore modem control lines, enable receiver
    tty.c_cflag &= ~CSIZE;                  // Clear the size bits
    tty.c_cflag |= CS8;                     // 8-bit data
    tty.c_cflag &= ~PARENB;                 // No parity
    tty.c_cflag &= ~CSTOPB;                 // 1 stop bit

    tcflush(serial_port_fd, TCIFLUSH);
    if (tcsetattr(serial_port_fd, TCSANOW, &tty) != 0)
    {
        perror("Error from tcsetattr");
        return;
    }
}

std::string MbotMain::mac_bytes_to_string(const uint8_t mac_address[6]) const
{
    std::stringstream ss;
    for (int i = 0; i < 6; ++i)
    {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(mac_address[i]);
        if (i != 5)
            ss << ':';
    }
    return ss.str();
}

void MbotMain::mac_string_to_bytes(const std::string & mac_address, 
                                         uint8_t       mac_bytes[6]) const
{
    std::stringstream ss(mac_address);
    int i = 0;
    while (ss.good() && i < 6)
    {
        std::string substr;
        getline(ss, substr, ':');
        mac_bytes[i] = std::stoi(substr, nullptr, 16);
        i++;
    }
}

void MbotMain::read_bytes(uint8_t * buffer, 
                          uint16_t  len) const
{
    size_t bytes_read = 0;
    while (bytes_read < len)
    {
        bytes_read += read(serial_port_fd, buffer + bytes_read, len - bytes_read);
    }
}

void MbotMain::read_mac_address(uint8_t  * mac_address, 
                                uint16_t * pkt_len) const
{
    uint8_t pkt_len_buf[2];
    read_bytes(pkt_len_buf, 2);
    *pkt_len = ((uint16_t)pkt_len_buf[1] << 8) | (uint16_t)pkt_len_buf[0];
    read_bytes(mac_address, MAC_ADDR_LEN);
}

void MbotMain::read_message(      uint8_t  * data_serialized, 
                            const uint16_t & message_len, 
                                  uint8_t  * data_checksum) const
{
    read_bytes(data_serialized, message_len);
    read_bytes(data_checksum, 1);
}

bool MbotMain::validate_message(const uint8_t  * const data_serialized, 
                                const uint16_t &       message_len, 
                                const uint8_t  &       data_checksum) const
{
    uint8_t cs_data = checksum(data_serialized, message_len);
    bool valid_message = (cs_data == data_checksum);
    return valid_message;
}

void MbotMain::encode_msg(const uint8_t  * const in_msg, 
                          const uint16_t &        in_msg_len, 
                          const uint16_t &        topic, 
                          const uint8_t  *        mac_addr, 
                                uint8_t  *        out_pkt) const
{
    // add mac address
    out_pkt[0] = SYNC_FLAG;
    out_pkt[1] = (uint8_t)((in_msg_len + ROS_PKG_LEN) % 255);
    out_pkt[2] = (uint8_t)((in_msg_len + ROS_PKG_LEN) >> 8);
    std::memcpy(out_pkt + 3, mac_addr, MAC_ADDR_LEN);

    // add ROS packet header
    out_pkt[9] = SYNC_FLAG;
    out_pkt[10] = VERSION_FLAG;
    out_pkt[11] = (uint8_t)(in_msg_len % 255);
    out_pkt[12] = (uint8_t)(in_msg_len >> 8);
    uint8_t cs1_addends[2] = {out_pkt[11], out_pkt[12]};
    out_pkt[13] = checksum(cs1_addends, 2);

    // add topic and message
    out_pkt[14] = (uint8_t)(topic % 255);
    out_pkt[15] = (uint8_t)(topic >> 8);
    std::memcpy(out_pkt + 16, in_msg, in_msg_len);
    uint8_t* cs2_addends = new uint8_t[in_msg_len + 2];
    cs2_addends[0] = out_pkt[14];
    cs2_addends[1] = out_pkt[15];
    std::memcpy(cs2_addends + 2, (uint8_t *)in_msg, in_msg_len);
    out_pkt[16 + in_msg_len] = checksum(cs2_addends, in_msg_len + 2);
    delete[] cs2_addends;
}

uint8_t MbotMain::checksum(const uint8_t * const addends, 
                           const int     &       len) const
{
    int sum = 0;
    for (int i = 0; i < len; i++)
    {
        sum += addends[i];
    }
    return 255 - ((sum) % 256);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MbotMain>("/dev/cu.usbserial14110"));
  rclcpp::shutdown();
  return 0;
}