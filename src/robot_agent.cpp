// Created by Indraneel on 22/01/21

#include "robot_agent.hpp"
#include "ssh_session.hpp"
#include <ros/console.h>
#include <memory>
#include <ros/package.h>
#include "boost/bind.hpp"
#include <boost/thread.hpp>

RobotAgent::RobotAgent(const std::string robot_id, const std::string ip_address, const std::string server_ip_addr,
                       const std::string path_to_code, const int feedback_port, const int control_port, const int feedback_freq, const int control_timeout) :

                                                                                                                                                              robot_id_(robot_id), ip_address_(ip_address), server_ip_addr_(server_ip_addr),bridgePtr(std::make_shared<ROSFeedbackBridge>(robot_id,nh_,feedback_freq)),

                                                                                                                                                              path_to_code_(path_to_code), feedback_port_(feedback_port), control_port_(control_port), control_timeout_ms_(control_timeout),
                                                                                                                                                              feedback_freq_hz_(feedback_freq), nh_("~" + robot_id), comm_channel_(robot_id, io_service, feedback_port, ip_address, control_port, bridgePtr), work(io_service),
                                                                                                                                                              odom_TF_pub(nh_)
{

    // @indraneel initialise communication

    // Set up khepera robot
    std::shared_ptr<SSHSession> sessionPtr(new SSHSession(ip_address_));
    if (!sessionPtr->initiateConnection())
    {
        agent_status_.setStatus(RobotStatus::ROBOT_STATUS_UNREACHABLE);
        ROS_WARN("Could not reach %s", &robot_id_[0]);
        return;
    }

    // Get path to config file
    std::string package_path = ros::package::getPath("robosar_agent_bringup");
    std::string shell = package_path + "/script/khepera_setup.sh";
    // Run setup script
    std::system(&(shell + " " + path_to_code_ + " " + ip_address_ + " " + server_ip_addr_ +
                  " " + std::to_string(feedback_port_) + " " + std::to_string(control_port_) + " " + std::to_string(feedback_freq_hz_) + " " + std::to_string(control_timeout_ms_))[0]);
    
    deadman_timer_ = nh_.createTimer(ros::Duration(1.0), boost::bind(&RobotAgent::timerCallback, this, _1));
    // Timer only starts once agent is up
    deadman_timer_.stop();
    timer_ptr_ = std::make_shared<ros::Timer>(deadman_timer_);
    comm_channel_.deadman_timer_ptr_  = timer_ptr_;


    // Create io_service background thread for udp server
    try
    {
        boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Exception: %s\n", e.what());
        agent_status_.setStatus(RobotStatus::ROBOT_STATUS_COMM_FAIL);
        return;
    }

    // Create ROS nodes for this agent
    control_subscriber_ = nh_.subscribe("control", 1, &RobotAgent::velocityCallback, this);

    status_ptr_ = std::make_shared<RobotStatus>(agent_status_);
    comm_channel_.status_ptr_ = status_ptr_;
}


/**
 * @brief Returns agent status
 * 
 * @param void
 * @return status_e
 * 
 */
RobotStatus::status_e RobotAgent::getAgentStatus()
{
    return agent_status_.getStatus();
}

std::string RobotAgent::getAgentStatusString() {
    return agent_status_.robotStatusStrVec[agent_status_.getStatus()];
}


/**
 * @brief Receives data from ROS and then sends it to robot using UDP
 * 
 * @param vel_msg twist message from ROS
 * @return void
 * 
 */
void RobotAgent::velocityCallback(const geometry_msgs::Twist &vel_msg)
{
    ROS_DEBUG("Sending velocity v:%f w:%f to %s!\n", vel_msg.linear.x, vel_msg.angular.z, &robot_id_[0]);
    // Convert m/s to mm/s because khepera code needs it in mm/s
    std::string command_msg = std::to_string(vel_msg.angular.z) + "x" + std::to_string(vel_msg.linear.x*1000.0);

    memcpy(comm_channel_.send_data_, command_msg.data(), command_msg.length());
    //async send
    comm_channel_.do_send(command_msg.length());
}

/**
 * @brief Called once deadman timer expires
 * 
 * @return void
 * 
 */
void RobotAgent::timerCallback(const ros::TimerEvent& timer_event) {
    agent_status_.setStatus(RobotStatus::ROBOT_STATUS_NO_HEARTBEAT);
    ROS_WARN("%s: NO HEARTBEAT", &robot_id_[0]);
    timer_ptr_->stop();
}