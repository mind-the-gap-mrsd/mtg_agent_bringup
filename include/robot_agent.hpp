// Created by Indraneel on 22/01/21

#ifndef ROBOT_AGENT_HPP
#define ROBOT_AGENT_HPP

#include <json/json.h>
#include <string.h>
#include <mutex>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include "async_udp_server.hpp"

class RobotAgent
{

public:
    ~RobotAgent() = default;
    RobotAgent(const std::string robot_id, const std::string ip_address, const std::string server_ip_addr,
               const std::string path_to_code, const int feedback_port, const int control_port, const int feedback_freq, const int control_timeout);

    typedef enum robotStatus
    {
        ROBOT_STATUS_ACTIVE,
        ROBOT_STATUS_INACTIVE,
        ROBOT_STATUS_IDLE,
        ROBOT_STATUS_COMM_FAIL,
        ROBOT_STATUS_UNREACHABLE
    } robotStatus_e;

    std::vector<std::string> robotStatusStrVec{
        "ROBOT_STATUS_ACTIVE",
        "ROBOT_STATUS_INACTIVE",
        "ROBOT_STATUS_IDLE",
        "ROBOT_STATUS_COMM_FAIL",
        "ROBOT_STATUS_UNREACHABLE"};

    robotStatus_e getAgentStatus();
    void velocityCallback(const geometry_msgs::Twist &vel_msg);

    const std::string path_to_code_;
    const std::string robot_id_;
    const std::string ip_address_;
    const std::string server_ip_addr_;
    const int feedback_port_;
    const int control_port_;
    const int feedback_freq_hz_;
    const int control_timeout_ms_;

private:
    robotStatus_e status;
    std::mutex m;
    boost::asio::io_service io_service;
    boost::asio::io_service::work work;
    udp_server comm_channel_;
    ros::NodeHandle nh_;
    ros::Publisher feedback_publisher_;
    ros::Subscriber control_subscriber_;
    ros::Timer deadman_timer_;
};

#endif