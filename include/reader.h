/*
 * ibeo_client.h
 * Copyright (C) 2017 Roman C. Podolski <mailto:roman.podolski@tum.de>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef LIDAR_READER_H_
#define LIDAR_READER_H_

#include "parser.h"
#include <boost/asio.hpp>
#include <gflags/gflags.h>
#include <list> // check if a list is the best option here
#include <string>
#include <boost/asio.hpp>
#include <chrono>
#include <glog/logging.h>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "custom_message.h"
#include <thread>

DECLARE_string(ibeo_lux_ip_address);
DECLARE_string(ibeo_lux_port);


using boost::asio::ip::tcp;

using namespace ibeo_driver;

class Reader {
public:

  explicit Reader(boost::asio::io_service &io, ros::NodeHandle nh);
  ~Reader();

  void init(boost::asio::io_service &io);

  std::list<LidarMessage> _messages;
  void close() { _socket.close();}
  std::list<LidarMessage> &get_messages() { return _messages; }

private:
  boost::asio::io_service &_io_service;
  // TODO(roman/sam): do we need the stand;
  boost::asio::io_service::strand _strand;
  boost::asio::deadline_timer _timer;
  tcp::resolver _resolver;
  tcp::socket _socket;
  Parser _parser;
  ros::NodeHandle nh;
  const double ibeo_frequency = 100.0;

  void do_connect();
  void do_sync();
  void do_read_header();
  void do_read_body();
};

#endif // LUX_CLIENT_H_
