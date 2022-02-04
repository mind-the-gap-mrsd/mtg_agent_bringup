// Created by Indraneel on 3/02/21

#ifndef ROS_FEEDBACK_BRIDGE_HPP
#define ROS_FEEDBACK_BRIDGE_HPP

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "robosar.pb.h"

class ROSFeedbackBridge
{

public:
    ROSFeedbackBridge(ros::NodeHandle nh):nh_(nh)
    {
        // Create ROS nodes for this agent
        feedback_publisher_ = nh_.advertise<std_msgs::Float32>("feedback", 1, true);
    }

    ~ROSFeedbackBridge() = default;

    void unpack_feedback_message(robosar_fms::SensorData feedback) {

        ROS_INFO("Unpacking message");
    }


private:
    ros::NodeHandle nh_;
    ros::Publisher feedback_publisher_;
};

#endif