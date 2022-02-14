#include "odom_node.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include "std_msgs/Int16.h"
#include <geometry_msgs/PoseStamped.h>

using namespace std;

nav_msgs::Odometry odomNew;
nav_msgs::Odometry odomOld;
#define SizeOfCovarianceMatrix 36

// Initial pose
const double initialX = 0.0;
const double initialY = 0.0;
const double initialTheta = 0.00000000001;
const double PI = 3.141592;

// Robot physical constants
const double TICKS_PER_REVOLUTION = 19456;              // For reference purposes.
const double wheel_distance = 0.10470;                  // Center of left tire to center of right tire
const double wheel_conversion_left = 0.13194 / 19456.0; // perimeter over pulses per rev
const double wheel_conversion_right = 0.13194 / 19456.0;

// Encoder value
int pos_left = 0;
int pos_right = 0;
int pos_left_prev = 0;
int pos_right_prev = 0;

// Publish a nav_msgs::Odometry message in quaternion format
void publish_quat(ros::Publisher &odom_data_pub_quat)
{

  tf2::Quaternion q;

  q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

  nav_msgs::Odometry quatOdom;
  quatOdom.header.stamp = odomNew.header.stamp;
  quatOdom.header.frame_id = "odom";
  quatOdom.child_frame_id = "base_link";
  quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
  quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
  quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
  quatOdom.pose.pose.orientation.x = q.x();
  quatOdom.pose.pose.orientation.y = q.y();
  quatOdom.pose.pose.orientation.z = q.z();
  quatOdom.pose.pose.orientation.w = q.w();
  quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
  quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
  quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
  quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
  quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
  quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

  for (int i = 0; i < SizeOfCovarianceMatrix; i++)
  {
    if (i == 0 || i == 7 || i == 14)
    {
      // diagonal elements
      quatOdom.pose.covariance[i] = .01;
    }
    else if (i == 21 || i == 28 || i == 35)
    {
      // roll pitch and yaw
      quatOdom.pose.covariance[i] += 0.1;
    }
    else
    {
      quatOdom.pose.covariance[i] = 0;
    }
  }

  odom_data_pub_quat.publish(quatOdom);
}

// Update odometry information
void update_odom(ros::Publisher &odom_data_pub)
{

  long delta_pos_left, delta_pos_right;
  float delta_left, delta_right, delta_theta, theta2;
  float delta_x, delta_y;

  delta_pos_left = pos_left - pos_left_prev;
  delta_pos_right = pos_right - pos_right_prev;
  delta_left = delta_pos_left * wheel_conversion_left;
  delta_right = delta_pos_right * wheel_conversion_right;
  delta_theta = (delta_right - delta_left) / wheel_distance;
  theta2 = odomNew.pose.pose.orientation.z + delta_theta * 0.5;
  delta_x = (delta_left + delta_right) * 0.5 * cosf(theta2);
  delta_y = (delta_left + delta_right) * 0.5 * sinf(theta2);

  // Calculate the new pose (x, y, and theta)
  odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + delta_x;
  odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + delta_y;
  odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z + delta_theta;

  // Prevent lockup from a single bad cycle
  if (isnan(odomNew.pose.pose.position.x) || isnan(odomNew.pose.pose.position.y) || isnan(odomNew.pose.pose.position.z))
  {
    odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
    odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
    odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
  }

  // Make sure theta stays in the correct range
  if (odomNew.pose.pose.orientation.z > PI)
  {
    odomNew.pose.pose.orientation.z -= 2 * PI;
  }
  else if (odomNew.pose.pose.orientation.z < -PI)
  {
    odomNew.pose.pose.orientation.z += 2 * PI;
  }
  else
  {
    // do nothing in correct range already
  }

  // Compute the velocity
  odomNew.header.stamp = ros::Time::now();
  odomNew.twist.twist.linear.x = 0.5 * (delta_left + delta_right) / (odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());
  odomNew.twist.twist.angular.z = delta_theta / (odomNew.header.stamp.toSec() - odomOld.header.stamp.toSec());

  // Save the pose data for the next cycle
  odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
  odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
  odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
  odomOld.header.stamp = odomNew.header.stamp;

  // Publish the odometry message
  odom_data_pub.publish(odomNew);

  pos_left_prev = pos_left;
  pos_right_prev = pos_right;
}
