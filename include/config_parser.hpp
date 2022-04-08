// Created by Indraneel on 22/01/21

#ifndef CONFIG_PARSER_HPP
#define CONFIG_PARSER_HPP

#include <json/json.h>
#include "robot_agent.hpp"
#include <memory>
#include "robosar_messages/agent_status.h"
class ConfigParser
{

public:
    ~ConfigParser() = default;
    ConfigParser();

    std::vector<std::shared_ptr<RobotAgent>> agents_vec;
    bool pubAgentInfo(robosar_messages::agent_status::Request  &req,
                     robosar_messages::agent_status::Response &res); 

private:
    static bool is_initialized_;
    bool simulation_flag;
    std::set<std::string> agent_names;

    void configSystemInit(Json::Value config);
    ros::NodeHandle nh;
    ros::ServiceServer sh;
    
};

#endif