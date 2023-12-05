#ifndef COMMS_H
#define COMMS_H

#define ROS_HEADER_LEN  7
#define ROS_FOOTER_LEN  1 
#define ROS_PKG_LEN     ROS_HEADER_LEN + ROS_FOOTER_LEN
#define SYNC_FLAG       0xff
#define VERSION_FLAG    0xfe
#define MAC_ADDR_LEN    6

enum message_topics{
    MBOT_TIMESYNC = 201, 
    MBOT_ODOMETRY = 210, 
    MBOT_ODOMETRY_RESET = 211,
    MBOT_VEL_CMD = 214,
    MBOT_IMU = 220,
    MBOT_ENCODERS = 221,
    MBOT_ENCODERS_RESET = 222,
    MBOT_MOTOR_PWM_CMD = 230,
    MBOT_MOTOR_VEL_CMD = 231,
    MBOT_MOTOR_VEL = 232,
    MBOT_MOTOR_PWM = 233,
    MBOT_VEL = 234
};

#endif