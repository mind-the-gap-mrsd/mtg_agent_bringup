#include <iostream>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include "robosar.pb.h"
#include <sensor_msgs/Imu.h>
#include <angles/angles.h>
#include "odom_node.hpp"
#include <nav_msgs/Odometry.h>
#include <chrono>
#include <thread>

using namespace std;


const int TICKS_PER_REVOLUTION = 19456;
const double perimeter = 0.13194;
int left_curr_tick = 0;
int right_curr_tick = 0;


int main(int argc, char **argv) {
    ros::init(argc, argv, "odom_test");
    ros::NodeHandle nh_;
    ros::Publisher odom_data_pub = nh_.advertise<nav_msgs::Odometry>("odom_data_euler", 100);

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

    vector<double> left_ticks;
    vector<double> right_ticks;

    // move in straight line
    // for (int i=0; i<TICKS_PER_REVOLUTION*5; i+=TICKS_PER_REVOLUTION) {
    //     left_ticks.push_back(left_curr_tick + i);
    //     right_ticks.push_back(right_curr_tick + i);
    // }

    // left_curr_tick = left_ticks.back();
    // right_curr_tick = right_ticks.back();

    // spin in circle
    for (int i=0; i<TICKS_PER_REVOLUTION*5; i+=TICKS_PER_REVOLUTION*5/50) {
        left_ticks.push_back(left_curr_tick + i);
        right_ticks.push_back(right_curr_tick);
    }

    // left_curr_tick = left_ticks.back();
    // right_curr_tick = right_ticks.back();

    // for (int i=0; i<TICKS_PER_REVOLUTION*3; i+=TICKS_PER_REVOLUTION*3/50) {
    //     left_ticks.push_back(left_curr_tick);
    //     right_ticks.push_back(right_curr_tick + i);
    // }

    // odometry estimate
    for (int i=0; i<left_ticks.size(); i++) {
        pos_left = left_ticks[i];
        pos_right = right_ticks[i];
        update_odom(odom_data_pub);
        ROS_INFO("x: %f", odomNew.pose.pose.position.x);
        ROS_INFO("y: %f", odomNew.pose.pose.position.y);
        ROS_INFO("th: %f", odomNew.pose.pose.orientation.z);

        this_thread::sleep_for(chrono::milliseconds(100));
    }

}