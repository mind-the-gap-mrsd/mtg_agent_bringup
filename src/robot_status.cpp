#include "robot_status.hpp"
#include "robot_agent.hpp"
std::atomic<RobotStatus::status_e> status_(RobotStatus::ROBOT_STATUS_ACTIVE);

RobotStatus::RobotStatus() {
    //status_ = ROBOT_STATUS_ACTIVE;
    
}


/**
 * @brief Returns agent status
 * 
 * @param void
 * @return status_e
 * 
 */
RobotStatus::status_e RobotStatus::getStatus()
{
    return status_;
}


void RobotStatus::setStatus(status_e new_status) {
    status_ = new_status;
    std_msgs::String msg;
    if(new_status==RobotStatus::ROBOT_STATUS_NO_HEARTBEAT)
    {
        msg.data = "0";
    }
    else
    {
        msg.data = "1";
    }
    RobotStatus::status_pub.publish(msg);
}