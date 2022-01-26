// Created by Indraneel on 22/01/21

#include "robot_agent.hpp"
#include "ssh_session.hpp"
#include <ros/console.h>
#include <memory>


RobotAgent::RobotAgent(const std::string robot_id, const std::string ip_address,std::string path_to_code,const int feedback_port, const int control_port) :
             robot_id_(robot_id), ip_address_(ip_address), path_to_code_(path_to_code), feedback_port_(feedback_port), control_port_(control_port) {


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


    status = ROBOT_STATUS_ACTIVE;
}

RobotAgent::robotStatus_e RobotAgent::getAgentStatus() {

    // @indraneel add mutex lock
    std::lock_guard<std::mutex> guard(m);
    return status;
}