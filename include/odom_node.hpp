#ifndef ODOM_NODE_HPP
#define ODOM_NODE_HPP

#include "ros/ros.h"
#include <nav_msgs/Odometry.h>

extern int pos_left;
extern int pos_right;
extern nav_msgs::Odometry odomNew;
extern nav_msgs::Odometry odomOld;

extern void publish_quat(ros::Publisher &odom_data_pub_quat); 

extern void update_odom(ros::Publisher &odom_data_pub);

#endif