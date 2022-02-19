// Created by Indraneel on 22/01/21

#ifndef ROBOT_AGENT_HPP
#define ROBOT_AGENT_HPP

#include <json/json.h>
#include <string.h>
#include <boost/asio.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "async_udp_server.hpp"
#include "ros_feedback_bridge.hpp"
#include "odom_tf.hpp"
#include "robot_status.hpp"

class RobotAgent
{

public:
    ~RobotAgent() = default;
    RobotAgent(const std::string robot_id, const std::string ip_address, const std::string server_ip_addr,
               const std::string path_to_code, const int feedback_port, const int control_port, const int feedback_freq, const int control_timeout);

    RobotStatus::status_e getAgentStatus();
    void velocityCallback(const geometry_msgs::Twist &vel_msg);
    void timerCallback(const ros::TimerEvent& timer_event);
    std::string getAgentStatusString();

    const std::string path_to_code_;
    const std::string robot_id_;
    const std::string ip_address_;
    const std::string server_ip_addr_;
    const int feedback_port_;
    const int control_port_;
    const int feedback_freq_hz_;
    const int control_timeout_ms_;

private:
    boost::asio::io_service io_service;
    boost::asio::io_service::work work;
    ros::NodeHandle nh_;
    std::shared_ptr<ROSFeedbackBridge> bridgePtr;
    udp_server comm_channel_;
    ros::Subscriber control_subscriber_;
    ros::Timer deadman_timer_;
    std::shared_ptr<ros::Timer> timer_ptr_;
    odomTF odom_TF_pub;
    RobotStatus agent_status_;
    std::shared_ptr<RobotStatus> status_ptr_;
};

#endif