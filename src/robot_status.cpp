#include "robot_status.hpp"

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
}