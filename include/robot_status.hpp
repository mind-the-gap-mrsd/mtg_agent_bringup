// Rachel Zheng
// 2/16/22

#ifndef ROBOT_STATUS_HPP
#define ROBOT_STATUS_HPP

#include <cstdlib>
#include <iostream>
#include <atomic>         // std::atomic, std::atomic_flag, ATOMIC_FLAG_INIT
#include <thread> 
#include <boost/asio.hpp>
#include <mutex>

class RobotStatus {

public:

    typedef enum Status
    {
        ROBOT_STATUS_ACTIVE,
        ROBOT_STATUS_INACTIVE,
        ROBOT_STATUS_IDLE,
        ROBOT_STATUS_COMM_FAIL,
        ROBOT_STATUS_UNREACHABLE,
        ROBOT_STATUS_NO_HEARTBEAT
    } status_e;
    
    ~RobotStatus(){};
    RobotStatus();
    

    status_e getAgentStatus();
    void setAgentStatus(status_e new_status);

private:
    status_e status_;
    // std::mutex m;
};
/*
class RobotStatus 
{
public:
    typedef enum Status
    {
        ROBOT_STATUS_ACTIVE,
        ROBOT_STATUS_INACTIVE,
        ROBOT_STATUS_IDLE,
        ROBOT_STATUS_COMM_FAIL,
        ROBOT_STATUS_UNREACHABLE,
        ROBOT_STATUS_NO_HEARTBEAT
    } status_e;

    ~RobotStatus(){};
    RobotStatus(){};

    status_e getAgentStatus(){return ROBOT_STATUS_ACTIVE;};
    void setAgentStatus(status_e new_status){};


private:
    status_e status_;
    std::mutex m;
};
*/
#endif