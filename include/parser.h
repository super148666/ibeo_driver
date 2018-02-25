/*
 * message.h
 * Copyright (C) 2017 Roman C. Podolski <mailto:roman.podolski@tum.de>
 *
 * Distributed under terms of the MIT license.
 */

#ifndef LIDAR_PARSER_H_
#define LIDAR_PARSER_H_

#include <boost/asio.hpp>
#include <boost/math/constants/constants.hpp>
#include <cstring>
#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include "custom_message.h"
#include <boost/endian/arithmetic.hpp>
#include <boost/exception/all.hpp>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <glog/logging.h>

using namespace ibeo_driver;

class Parser {
 public:
  enum { header_length = 24 };
  enum { max_body_length = 20000 };

  Parser() : body_length_(0) {}
  virtual ~Parser() {}

  const boost::asio::streambuf &buffer() const { return buffer_; }
  boost::asio::streambuf &buffer() { return buffer_; }

  std::size_t length() const { return header_length + body_length_; };

  size_t body_length() { return body_length_; };

  void body_length(size_t length) {
    DLOG_IF(FATAL, body_length_ > max_body_length)
        << "Max body lenght to small, increase!";
    body_length_ = length;
  }

  bool decode_header();
  bool decode_body();
  ibeo_driver::LidarMessage &get_message();

 private:
  int contour_offset;
  
  std::size_t body_length_;
  uint16_t data_type_;
  ibeo_driver::LidarMessage lidar_message_;

  boost::asio::streambuf buffer_;

  // SCAN DATA RELATED MEMBER FUNCTIONS
  void process_scan_data(const char *data);

  void process_mounting_position(
      ibeo_driver::MountingPosition *m_pos, const char *data);

  void process_scan_point(ibeo_driver::ScanPoint *point,
                          const char *data);

  // OBJECT DATA RELATED MEMBER FUNCTIONS

  void process_object_data(const char *data);

  void process_object(ibeo_driver::Object *object,
                      const char *data);

  void process_reference_point(
      ibeo_driver::ReferencePoint *rf, const char *data);

  void process_bounding_box(ibeo_driver::BoundingBox *bb,
                            const char *data);

  void process_object_box(ibeo_driver::ObjectBox *ob,
                          const char *data);

  void process_absolute_velocity(
      ibeo_driver::AbsoluteVelocity *av, const char *data);

  void process_point_2D(ibeo_driver::Point2D *p, const char *data);
  void process_size_2D(ibeo_driver::Size2D *s, const char *data);

  // ERROR DATA RELATED MEMBER FUNCTIONS

  void process_error_data(const char *data);
  void process_scanner_status(boost::endian::little_uint16_t status);
  void process_error_register_1(boost::endian::little_uint16_t reg);
  void process_error_register_2(boost::endian::little_uint16_t reg);
  void process_warning_register_1(boost::endian::little_uint16_t reg);
  void process_warning_register_2(boost::endian::little_uint16_t reg);
};

class wrong_layer : public boost::exception, public std::exception {
 private:
  // const char *what_;

 public:
  wrong_layer(uint8_t layer) {
    // what+
    // std::sprintf(what_, "invalid layer %i >= 4 was received", layer);
    // TODO(roman)
  }
  const char *what() const noexcept { return "wrong layer was received: "; }
};
typedef boost::error_info<wrong_layer, std::string> errmsg_info;


class bad_classification : public boost::exception, public std::exception {
  public:
    bad_classification(uint8_t classification) {
    }
    const char *what() const noexcept { return "wrong classification was recieved";}
};

#endif  // LIDAR_PARSER_H_
