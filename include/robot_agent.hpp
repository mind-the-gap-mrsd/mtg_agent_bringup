// Created by Indraneel on 22/01/21

#ifndef ROBOT_AGENT_H
#define ROBOT_AGENT_H

#include <json/json.h>
#include <string.h>
#include <mutex>


class RobotAgent {

    public:

        ~RobotAgent() = default;
        RobotAgent(const std::string robot_id, const std::string ip_address, std::string path_to_code, const int feedback_port, const int control_port);

        typedef enum robotStatus
        {
            ROBOT_STATUS_ACTIVE,
            ROBOT_STATUS_INACTIVE,
            ROBOT_STATUS_IDLE
        } robotStatus_e;

        robotStatus_e getAgentStatus();
        
        const std::string path_to_code_;
        const std::string robot_id_;
        const std::string ip_address_;
        const int feedback_port_;
        const int control_port_;

    private:
        robotStatus_e status;
        std::mutex m;


};

#endif