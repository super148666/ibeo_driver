/*
 * ibeo_client.cc
 * Copyright (C) 2017 Roman C. Podolski <mailto:roman.podolski@tum.de>
 *
 * For details on the implementation please look at the interface definition
 * provided in the datasheet of the ibeo LUX
 *
 * Distributed under terms of the MIT license.
 */



// static bool validate_IP_address(const char *flagname,
// const std::string &value) {
// struct sockaddr_in sa;
// // TODO(roman): use boost to validate ip address
// if (inet_pton(AF_INET, value.c_str(), &(sa.sin_addr)) == 1) return true;
// LOG(ERROR) << "Invalid value for " << static_cast<std::string>(flagname)
// << ":" << value;
// return false;
// }
#include "reader.h"

;DEFINE_string(ibeo_lux_ip_address, "192.168.2.4",
              "ip address of the Ibeo Laser scanner");
// DEFINE_validator(ibeo_ip_address_2, &validate_IP_address);
;DEFINE_double(ibeo_lux_frequency, 100.0, "Maximum Scan frequency for Ibeo Lux");

static bool validate_port(const char *flagname, int32_t value) {
  if (value > 0 && value < 32768)
    return true;
  LOG(ERROR) << "Invalid value for " << static_cast<std::string>(flagname)
             << ":" << value;
  return false;
}

DEFINE_string(ibeo_lux_port, "12002", "port of the ibeo laser scanner");
// DEFINE_validator(ibeo_lux_port, &validate_port);

Reader::Reader(boost::asio::io_service &io, ros::NodeHandle nh) // NOLINT(runtime/references)
    : _io_service(io),
      _resolver(io),
      _socket(io),
      _strand(io),
      _timer(io,
             boost::posix_time::millisec(1000.0 / ibeo_frequency)),
      nh(nh) {
  do_connect();
}

Reader::~Reader() { 
	close();
	
	 }

void Reader::do_connect() {
  tcp::resolver::query q{FLAGS_ibeo_lux_ip_address, FLAGS_ibeo_lux_port};
  ROS_INFO_STREAM("do connect");
  _resolver.async_resolve(
      q, [this](boost::system::error_code ec,
                boost::asio::ip::tcp::resolver::iterator it) {
		LOG_IF(FATAL, ec) << boost::system::system_error(ec).what(); 
		_socket.async_connect(*it, [this](boost::system::error_code ec) 
		{
          LOG_IF(FATAL, ec) << boost::system::system_error(ec).what();
          do_sync();
        });
      });
}

void Reader::do_sync() {
  std::string magic_word = {static_cast<char>(0xAF), static_cast<char>(0xFE),
                            static_cast<char>(0xC0), static_cast<char>(0xC2)};
  ROS_INFO_STREAM("sync read");
  boost::asio::async_read_until(
      _socket, _parser.buffer(), magic_word,
      [this](boost::system::error_code ec, std::size_t n) {
        LOG_IF(FATAL, ec) << boost::system::system_error(ec).what();
        _parser.buffer().consume(n - 4);
        do_read_header();
      });
}

void Reader::do_read_header() {
	ROS_INFO_STREAM("do read header");
  boost::asio::async_read(
      _socket, _parser.buffer().prepare(Parser::header_length),
      _strand.wrap([this](boost::system::error_code ec, std::size_t n) {
        CHECK_EQ(n, Parser::header_length);
        LOG_IF(FATAL, ec) << boost::system::system_error(ec).what();
        _parser.buffer().commit(n);
        if (_parser.decode_header()) {
          do_read_body();
        } else {
          LOG(WARNING) << "Lost Sync";
          _timer.async_wait([this](boost::system::error_code ec) {
            LOG_IF(FATAL, ec) << boost::system::system_error(ec).what();
            _timer.expires_from_now(
                boost::posix_time::millisec(1000.0 / ibeo_frequency));
            do_sync();
          });
        }
      }));
}

void Reader::do_read_body() {
	ROS_INFO_STREAM("do read body");
  boost::system::error_code ec;
  boost::asio::async_read(
      _socket, _parser.buffer().prepare(_parser.body_length()),
      _strand.wrap([this](boost::system::error_code ec, std::size_t n) {
        CHECK_EQ(n, _parser.body_length());
        LOG_IF(FATAL, ec) << boost::system::system_error(ec).what();
        _parser.buffer().commit(n);
        if (_parser.decode_body()) {
          _messages.push_back(_parser.get_message());
        } else {
          LOG(WARNING) << "Lidar not in sync";
        }
        _timer.async_wait([this](boost::system::error_code ec) {
          LOG_IF(FATAL, ec) << boost::system::system_error(ec).what();
          _timer.expires_from_now(
              boost::posix_time::millisec(1000.0 / ibeo_frequency));
          do_sync();
        });
      }));
}
