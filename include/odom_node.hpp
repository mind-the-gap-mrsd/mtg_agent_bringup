#ifndef ODOM_NODE_HPP
#define ODOM_NODE_HPP


// Calculate the distance the left wheel has traveled since the last cycle
void Calc_Left(const int leftCount);

// Calculate the distance the right wheel has traveled since the last cycle
void Calc_Right(const int rightCount);


#endif