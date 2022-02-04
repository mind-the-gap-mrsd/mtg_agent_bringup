// Created by Indraneel on 3/02/21

#ifndef ROS_FEEDBACK_BRIDGE_HPP
#define ROS_FEEDBACK_BRIDGE_HPP

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "robosar.pb.h"
#include <sensor_msgs/Imu.h>
#include <angles/angles.h>

class ROSFeedbackBridge
{

public:
    ROSFeedbackBridge(ros::NodeHandle nh):nh_(nh)
    {
        khepera_frame = "base_link";
        // Create ROS nodes for this agent
        imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("feedback/IMU", 1, true);
        
    }

    ~ROSFeedbackBridge() = default;

    void unpack_feedback_message(robosar_fms::SensorData feedback) {

        ROS_INFO("Unpacking message");
        sensor_msgs::Imu imu_msg = sensor_msgs::Imu();
        imu_msg.header.frame_id = khepera_frame;
        imu_msg.header.seq = feedback.seq_id();

        //@indraneel convert to m/s^2
        imu_msg.linear_acceleration.x = feedback.accel_data().acc_x();
        imu_msg.linear_acceleration.y = feedback.accel_data().acc_y();
        imu_msg.linear_acceleration.z = feedback.accel_data().acc_z();

        imu_msg.angular_velocity.x = angles::from_degrees(feedback.gyro_data().gyro_x());
        imu_msg.angular_velocity.y = angles::from_degrees(feedback.gyro_data().gyro_y());
        imu_msg.angular_velocity.z = angles::from_degrees(feedback.gyro_data().gyro_z());
        imu_publisher_.publish(imu_msg);

    }


private:
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    std::string khepera_frame;
};

#endif