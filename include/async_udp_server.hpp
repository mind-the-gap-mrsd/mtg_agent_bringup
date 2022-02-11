//
// async_udp_echo_server.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2014 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#ifndef ASYNC_UDP_SERVER_HPP
#define ASYNC_UDP_SERVER_HPP

#include <cstdlib>
#include <iostream>
#include <boost/asio.hpp>
#include <ros/console.h>
#include "robosar.pb.h"
#include "ros_feedback_bridge.hpp"

using boost::asio::ip::udp;

class udp_server
{
public:
  udp_server(boost::asio::io_service &io_service, short port, std::string remote_ip_address, int remote_port, std::shared_ptr<ROSFeedbackBridge> bridgePtr)
      : socket_(io_service, udp::endpoint(udp::v4(), port)),bridgePtr_(bridgePtr) 
  {
    // Create remote endpoint
    boost::system::error_code myError;

    boost::asio::ip::address IP;
    IP = boost::asio::ip::address::from_string(remote_ip_address, myError);

    remote_endpoint_.address(IP);
    remote_endpoint_.port(remote_port);

    do_receive();
  }

  void do_receive()
  {
    socket_.async_receive_from(
        boost::asio::buffer(receive_data_, max_length), remote_endpoint_,
        [this](boost::system::error_code ec, std::size_t bytes_recvd) {
          if (!ec && bytes_recvd > 0)
          {
            //do_send(bytes_recvd);
            // Do something with received data
            ROS_DEBUG("Received %ld bytes of data!", bytes_recvd);

            // Unpack this data
            robosar_fms::SensorData feedback;
            if(!feedback.ParseFromArray(receive_data_,bytes_recvd))
            {
              ROS_ERROR("Failed to parse feedback\n");
            }
            else
            {
              bridgePtr_->unpack_feedback_message(feedback);      
            }     

            do_receive();
          }
        });
  }

  void do_send(std::size_t length)
  {
    socket_.async_send_to(
        boost::asio::buffer(send_data_, length), remote_endpoint_,
        [this](boost::system::error_code ec, std::size_t bytes_sent) {
          if (!ec)
          {
            ROS_DEBUG("Successfully sent %ld bytes \n", bytes_sent);
          }
          else
          {
            ROS_ERROR("Failed to send data\n");
          }
        });
  }

  enum
  {
    max_length = 2000
  };
  char send_data_[max_length];
  char receive_data_[max_length];

private:
  udp::socket socket_;
  udp::endpoint remote_endpoint_;
  std::shared_ptr<ROSFeedbackBridge> bridgePtr_;
};

#endif