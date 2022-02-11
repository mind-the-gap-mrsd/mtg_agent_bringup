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
#include "odom_node.hpp"
#include <nav_msgs/Odometry.h>
#include <mutex>

// TODO
// Cross check variable reshuffling by neel
// Can the odometry freq by derived from feedback frequency?
// Put mutex locks on pos_left and pos_right
class ROSFeedbackBridge
{

public:
    ROSFeedbackBridge(ros::NodeHandle nh):nh_(nh),node_alive_(true),odom_freq_hz(5),khepera_frame("base_link")
    {

        // Create ROS nodes for this agent
        imu_publisher_ = nh_.advertise<sensor_msgs::Imu>("feedback/IMU", 1, true);
        odom_data_pub = nh_.advertise<nav_msgs::Odometry>("odom_data_euler", 100);
        odom_data_pub_quat = nh_.advertise<nav_msgs::Odometry>("odom_data_quat", 100);
        lrf_publisher_ = nh_.advertise<sensor_msgs::LaserScan>("feedback/scan", 1, true);

        //// Start Odom node
        odom_delay_ms = (int)((1.0/(double)(odom_freq_hz))*1000.0);
        odom_thread_ = std::thread(&ROSFeedbackBridge::runOdometry, this);
        
    }

    ~ROSFeedbackBridge()
    {
        ROS_INFO("Killing feedback bridge");
        node_alive_ = false;
        odom_thread_.join();
    }

    void unpack_feedback_message(robosar_fms::SensorData feedback) {

        ROS_DEBUG("Unpacking message");

        //IMU
        sensor_msgs::Imu imu_msg;
        
        imu_msg.header.frame_id = khepera_frame;
        imu_msg.header.seq = feedback.seq_id();
        imu_msg.header.stamp = ros::Time::now();

        //@indraneel convert to m/s^2
        imu_msg.linear_acceleration.x = feedback.accel_data().acc_x();
        imu_msg.linear_acceleration.y = feedback.accel_data().acc_y();
        imu_msg.linear_acceleration.z = feedback.accel_data().acc_z();

        imu_msg.angular_velocity.x = angles::from_degrees(feedback.gyro_data().gyro_x());
        imu_msg.angular_velocity.y = angles::from_degrees(feedback.gyro_data().gyro_y());
        imu_msg.angular_velocity.z = angles::from_degrees(feedback.gyro_data().gyro_z());
        
        imu_publisher_.publish(imu_msg);

        // encoder callback
        {
            std::lock_guard<std::mutex> guard(mtx);
            pos_left = feedback.count_data().left();
            pos_right = feedback.count_data().right();
            ROS_INFO("Left ticks: %d", pos_left);
            ROS_INFO("Right ticks: %d", pos_right);
        }

        // LaserScan
        sensor_msgs::LaserScan lrf_msg;

        lrf_msg.header.frame_id = khepera_frame;
        lrf_msg.header.stamp = ros::Time::now();
        lrf_msg.header.seq = feedback.seq_id();
        // verified from URG-04LX-UG01 specification datasheet
        lrf_msg.angle_min = -2.0923497676849365;
        lrf_msg.angle_max = 2.0923497676849365;
        lrf_msg.angle_increment = 0.006135923322290182;
        lrf_msg.time_increment = 9.765627328306437e-05;
        lrf_msg.scan_time = 0.10000000149011612;
        lrf_msg.range_min = 0.019999999552965164;
        lrf_msg.range_max = 5.599999904632568;
        robosar_fms::LaserScanner lrf_feedback = feedback.lrf_data();
        //ROS_INFO("Lrf data size : %d\n",lrf_feedback.values_size());
        for(int i=0;i<lrf_feedback.values_size();i++)
        {
            // mm to metres
            lrf_msg.ranges.push_back((float)(lrf_feedback.values(i))/1000.0f);
        }

        lrf_publisher_.publish(lrf_msg);
    }

    void runOdometry()
    {
        //ROS_INFO("Spinning odometry with delay %d ms",odom_delay_ms);

        // Set the data fields of the odometry message
        odomNew.header.frame_id = "odom";
        odomNew.pose.pose.position.z = 0;
        odomNew.pose.pose.orientation.x = 0;
        odomNew.pose.pose.orientation.y = 0;
        odomNew.twist.twist.linear.x = 0;
        odomNew.twist.twist.linear.y = 0;
        odomNew.twist.twist.linear.z = 0;
        odomNew.twist.twist.angular.x = 0;
        odomNew.twist.twist.angular.y = 0;
        odomNew.twist.twist.angular.z = 0;
        odomOld.pose.pose.position.x = 0;
        odomOld.pose.pose.position.y = 0;
        odomOld.pose.pose.orientation.z = 0;

        while(node_alive_)
        {
            {
                std::lock_guard<std::mutex> guard(mtx);
                update_odom(odom_data_pub);
            }
            publish_quat(odom_data_pub_quat);

            std::this_thread::sleep_for(std::chrono::milliseconds(odom_delay_ms));   
        }
    }


private:
    ros::NodeHandle nh_;
    ros::Publisher imu_publisher_;
    ros::Publisher odom_data_pub;
    ros::Publisher odom_data_pub_quat;
    ros::Publisher lrf_publisher_;
    std::string khepera_frame;
    std::thread odom_thread_;
    bool node_alive_;
    int odom_freq_hz;
    int odom_delay_ms;
    std::mutex mtx;
};

#endif