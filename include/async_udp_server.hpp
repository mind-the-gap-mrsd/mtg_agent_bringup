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

using boost::asio::ip::udp;

class udp_server
{
public:
  udp_server(boost::asio::io_service& io_service, short port)
    : socket_(io_service, udp::endpoint(udp::v4(), port))
  {
    do_receive();
  }

  void do_receive()
  {
    socket_.async_receive_from(
        boost::asio::buffer(data_, max_length), sender_endpoint_,
        [this](boost::system::error_code ec, std::size_t bytes_recvd)
        {
          if (!ec && bytes_recvd > 0)
          {
            //do_send(bytes_recvd);
            // Do something with received data
            ROS_INFO("Received some data!");
          }
        });
  }

  void do_send(std::size_t length, std::string remote_ip_address, int remote_port)
  {
    // Create remote endpoint
    boost::system::error_code myError;

    boost::asio::ip::address IP;
    IP = boost::asio::ip::address::from_string(remote_ip_address, myError); 

    udp::endpoint remote_endpoint_;
    remote_endpoint_.address(IP);
    remote_endpoint_.port(remote_port);


    socket_.async_send_to(
        boost::asio::buffer(data_, length), remote_endpoint_,
        [this](boost::system::error_code ec, std::size_t bytes_sent)
        {
          //do_receive();
          if(!ec)
          {
            ROS_INFO("Successfully sent %ld bytes \n",bytes_sent);
          }
          else
          {
            ROS_ERROR("Failed to send data\n");
          }
        });
  }

private:
  udp::socket socket_;
  udp::endpoint sender_endpoint_;
  enum { max_length = 1024 };
  char data_[max_length];
};

#endif