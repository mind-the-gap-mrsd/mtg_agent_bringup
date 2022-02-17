#include "robot_status.hpp"

RobotStatus::RobotStatus() {
    // std::lock_guard<std::mutex> guard(m);
    status_ = ROBOT_STATUS_ACTIVE;
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
    // @indraneel add mutex lock
    // std::lock_guard<std::mutex> guard(m);
    return status_;
}


void RobotStatus::setStatus(status_e new_status) {
    // std::lock_guard<std::mutex> guard(m);
    status_ = new_status;
}