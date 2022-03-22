// Created by Indraneel on 22/01/21

#include "config_parser.hpp"
#include <ros/package.h>
#include <assert.h>
#include <memory.h>
#include <ros/console.h>
#include <fstream>
#include <sys/types.h>
#include <sys/stat.h>
#include "robot_status.hpp"
#include "easylogging++.h"

bool ConfigParser::is_initialized_ = false;
INITIALIZE_EASYLOGGINGPP
std::string robot_active_status = "ROBOT_STATUS_ACTIVE";
ConfigParser::ConfigParser()
{

    // Get path to config file
    std::string package_path = ros::package::getPath("robosar_agent_bringup");
    std::string config_file_path_ = package_path + "/config/user_config.json";

    ROS_INFO("Looking for config file in path %s", &config_file_path_[0]);

    Json::Reader reader;
    Json::Value config;
    std::filebuf fb;
    bool parsing_status = false;

    if (fb.open(config_file_path_, std::ios::in))
    {
        std::istream config_stream(&fb);
        parsing_status = reader.parse(config_stream, config, false);
    }

    // Check parsing result
    assert(parsing_status);

    ROS_INFO("Config file parsed successfully, chosen config : %s", &config["config_selection"].asString()[0]);

    configSystemInit(config);
    sh = nh.advertiseService("agent_status", &ConfigParser::PubAgentInfo, this);
}
/**
 * @brief initialises system based on the user config file
 * 
 * @param config user config file
 * @return void
 * 
 */

/**
 * @brief initialises system based on the user config file
 * 
 * @param config user config file
 * @return void
 * 
 */

void ConfigParser::configSystemInit(Json::Value config)
{

    std::string path_to_khepera_code = config["path_to_khepera_code"].asString();
    std::string server_ip_add = config["server_ip_address"].asString();
    // std::string robot_id = config[config1[0]]
    // check if this file exists
    struct stat info;
    assert(stat(&path_to_khepera_code[0], &info) == 0);

    config = config[config["config_selection"].asString()];

    // Iterate through all agents
    int it = 0;
    for (const auto &itr : config)
    {

        it++;
        Json::Value agent_config;
        agent_config = itr;

        // Create new Agent
        std::shared_ptr<RobotAgent> agentPtr(new RobotAgent("agent" + std::to_string(it),
                                                            agent_config["ip_address"].asString(),
                                                            server_ip_add,
                                                            path_to_khepera_code,
                                                            agent_config["feedback_port"].asInt(),
                                                            agent_config["control_port"].asInt(),
                                                            agent_config["feedback_freq_hz"].asInt(),
                                                            agent_config["control_timeout_ms"].asInt()));
        // Check if agent is alive
        if (agentPtr->getAgentStatus() == RobotStatus::ROBOT_STATUS_ACTIVE)
        {
            // Add it to vector
            agents_vec.push_back(agentPtr);
        }
        //print agent status for awareness
        ROS_INFO("%s\n", &("AGENT" + std::to_string(it) + " :" + agentPtr->getAgentStatusString())[0]);
    }

    ROS_INFO("Number of agents online : %ld/%d\n", agents_vec.size(), it);
}

bool ConfigParser::PubAgentInfo(robosar_messages::agent_status::Request  &req, robosar_messages::agent_status::Response &res)
{
    std::vector<std::string> status;
    try
    {
        int itr = 0;
        for (auto agent : agents_vec)
        {
            
            if(std::strcmp(agent->getAgentStatusString().c_str(),robot_active_status.c_str())==0)
            {
                status.push_back(agent->robot_id_);
            }
            itr++;

        }
        itr = 0;
        // status.push_back("AGENT_"+std::to_string(itr));
        // status.push_back("DEAD");
        res.agents_active = status;
        return true;
    }
    catch(const std::exception& e)
    {
        std::cout<<e.what();
        return false;
    }
}