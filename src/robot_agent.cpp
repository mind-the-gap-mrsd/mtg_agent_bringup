// Created by Indraneel on 22/01/21

#include "robot_agent.hpp"
#include "ssh_session.hpp"
#include <ros/console.h>
#include <memory>
#include <ros/package.h>
#include "async_udp_server.hpp"


RobotAgent::RobotAgent(const std::string robot_id, const std::string ip_address,const std::string server_ip_addr, 
                            const std::string path_to_code,const int feedback_port, const int control_port, const int feedback_freq) :
             robot_id_(robot_id), ip_address_(ip_address), server_ip_addr_(server_ip_addr), 
             path_to_code_(path_to_code), feedback_port_(feedback_port), control_port_(control_port), feedback_freq_hz_(feedback_freq) {


    // @indraneel initialise communication
    std::lock_guard<std::mutex> guard(m);
    
    // Set up khepera robot
    std::shared_ptr<SSHSession> sessionPtr (new SSHSession(ip_address_)); 
    if(!sessionPtr->initiateConnection())
    {
        status = ROBOT_STATUS_UNREACHABLE;
        ROS_WARN("Could not reach %s",&robot_id_[0]);
        return;
    }

     // Get path to config file
    std::string package_path = ros::package::getPath("robosar_agent_bringup");
    std::string shell = package_path + "/script/khepera_setup.sh";
    // Run setup script
    std::system(&(shell + " " + path_to_code_ + " " + ip_address_ + " " + server_ip_addr_ +
                         " " + std::to_string(feedback_port_) + " " + std::to_string(control_port_) + " " +  std::to_string(feedback_freq_hz_))[0]);

    // Create UDP server for feedback and control
    try 
    {
        udp_server s(io_service, feedback_port_);

        io_service.run();
    }
    catch (std::exception& e)
    {
        ROS_ERROR("Exception: %s\n",e.what());
        status = ROBOT_STATUS_COMM_FAIL;
        return;
    }


    status = ROBOT_STATUS_ACTIVE;
}

RobotAgent::robotStatus_e RobotAgent::getAgentStatus() {

    // @indraneel add mutex lock
    std::lock_guard<std::mutex> guard(m);
    return status;
}