// Created by Indraneel on 22/01/21

#include "robot_agent.hpp"
#include "ssh_session.hpp"
#include <ros/console.h>
#include <memory>
#include <ros/package.h>
#include <boost/thread.hpp>

RobotAgent::RobotAgent(const std::string robot_id, const std::string ip_address, const std::string server_ip_addr,
                       const std::string path_to_code, const int feedback_port, const int control_port, const int feedback_freq, const int control_timeout) :

                                                                                                                                                              robot_id_(robot_id), ip_address_(ip_address), server_ip_addr_(server_ip_addr),bridgePtr(std::make_shared<ROSFeedbackBridge>(nh_,feedback_freq)),

                                                                                                                                                              path_to_code_(path_to_code), feedback_port_(feedback_port), control_port_(control_port), control_timeout_ms_(control_timeout),
                                                                                                                                                              feedback_freq_hz_(feedback_freq), nh_("~" + robot_id), comm_channel_(io_service, feedback_port, ip_address, control_port,bridgePtr), work(io_service),
                                                                                                                                                              odom_TF_pub(nh_)
{

    // @indraneel initialise communication
    std::lock_guard<std::mutex> guard(m);

    // Set up khepera robot
    std::shared_ptr<SSHSession> sessionPtr(new SSHSession(ip_address_));
    if (!sessionPtr->initiateConnection())
    {
        status = ROBOT_STATUS_UNREACHABLE;
        ROS_WARN("Could not reach %s", &robot_id_[0]);
        return;
    }

    // Get path to config file
    std::string package_path = ros::package::getPath("robosar_agent_bringup");
    std::string shell = package_path + "/script/khepera_setup.sh";
    // Run setup script
    std::system(&(shell + " " + path_to_code_ + " " + ip_address_ + " " + server_ip_addr_ +
                  " " + std::to_string(feedback_port_) + " " + std::to_string(control_port_) + " " + std::to_string(feedback_freq_hz_) + " " + std::to_string(control_timeout_ms_))[0]);

    // Create io_service background thread for udp server
    try
    {
        boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
    }
    catch (std::exception &e)
    {
        ROS_ERROR("Exception: %s\n", e.what());
        status = ROBOT_STATUS_COMM_FAIL;
        return;
    }

    // Create ROS nodes for this agent
    control_subscriber_ = nh_.subscribe("control", 1, &RobotAgent::velocityCallback, this);

    //@indraneel TODO Deadman timer for heartbeat

    status = ROBOT_STATUS_ACTIVE;
}

/**
 * @brief Returns agent status
 * 
 * @param void
 * @return robotStatus_e
 * 
 */
RobotAgent::robotStatus_e RobotAgent::getAgentStatus()
{

    // @indraneel add mutex lock
    std::lock_guard<std::mutex> guard(m);
    return status;
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