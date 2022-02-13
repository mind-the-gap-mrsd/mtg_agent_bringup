// Created by Indraneel on 3/02/21

#ifndef ROS_FEEDBACK_BRIDGE_HPP
#define ROS_FEEDBACK_BRIDGE_HPP

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "robosar.pb.h"
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <angles/angles.h>

class ROSFeedbackBridge
{

public:
    ROSFeedbackBridge(ros::NodeHandle nh):nh_(nh)
    {
        khepera_frame = "base_link";
        // Create ROS nodes for this agent
        imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("feedback/IMU", 1, true);
        lrf_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("feedback/scan", 1, true);
        
    }

    ~ROSFeedbackBridge() = default;

    void unpack_feedback_message(robosar_fms::SensorData* feedback) {

        ROS_DEBUG("Unpacking message");

        //IMU
        sensor_msgs::Imu imu_msg;
        
        imu_msg.header.frame_id = khepera_frame;
        imu_msg.header.seq = feedback->seq_id();
        imu_msg.header.stamp = ros::Time::now();

        //@indraneel convert to m/s^2
        imu_msg.linear_acceleration.x = feedback->accel_data().acc_x();
        imu_msg.linear_acceleration.y = feedback->accel_data().acc_y();
        imu_msg.linear_acceleration.z = feedback->accel_data().acc_z();

        imu_msg.angular_velocity.x = angles::from_degrees(feedback->gyro_data().gyro_x());
        imu_msg.angular_velocity.y = angles::from_degrees(feedback->gyro_data().gyro_y());
        imu_msg.angular_velocity.z = angles::from_degrees(feedback->gyro_data().gyro_z());
        
        imu_publisher_.publish(imu_msg);

        // LaserScan
        sensor_msgs::LaserScan lrf_msg;

        lrf_msg.header.frame_id = khepera_frame;
        lrf_msg.header.stamp = ros::Time::now();
        lrf_msg.header.seq = feedback->seq_id();
        // verified from URG-04LX-UG01 specification datasheet
        lrf_msg.angle_min = -2.0923497676849365;
        lrf_msg.angle_max = 2.0923497676849365;
        lrf_msg.angle_increment = 0.006135923322290182;
        lrf_msg.time_increment = 9.765627328306437e-05;
        lrf_msg.scan_time = 0.10000000149011612;
        lrf_msg.range_min = 0.019999999552965164;
        lrf_msg.range_max = 5.599999904632568;
        robosar_fms::LaserScanner lrf_feedback = feedback->lrf_data();
        //ROS_INFO("Lrf data size : %d\n",lrf_feedback.values_size());
        for(int i=0;i<lrf_feedback.values_size();i++)
        {
            // mm to metres
            lrf_msg.ranges.push_back((float)(lrf_feedback.values(i))/1000.0f);
        }

        lrf_publisher_.publish(lrf_msg);


    }


private:
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    ros::Publisher lrf_publisher_;
    std::string khepera_frame;
};

#endif