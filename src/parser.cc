/*
 * message_parser.cc
 * Copyright (C) 2017 Roman C. Podolski <mailto:roman.podolski@tum.de>
 *
 * Distributed under terms of the MIT license.
 */
#include "parser.h"

using boost::endian::big_uint16_t;
using boost::endian::big_uint32_t;
using boost::endian::big_uint64_t;
using boost::endian::little_uint8_t;
using boost::endian::little_uint16_t;
using boost::endian::little_uint64_t;
using boost::endian::little_int16_t;
using std::istream;
using std::memcpy;


bool Parser::decode_header() {
	ROS_INFO_STREAM("decode header");
  char header[header_length] = ""; // allocate and clear data for header
  istream is(&buffer_);
  is.read(header, header_length);

  // FILE *f = fopen("header_dump.bin", "wb");
  // fwrite(header, header_length, 1, f);
  // fclose(f);

  big_uint32_t magic_word;
  memcpy(&magic_word, &header[0], sizeof(magic_word));
  CHECK_EQ(magic_word, 0xAFFEC0C2); // sanity check

  big_uint32_t size_of_cur_msg;
  memcpy(&size_of_cur_msg, &header[8], sizeof(size_of_cur_msg));
  CHECK_GT(size_of_cur_msg, 0) << "No message body";
  body_length_ = size_of_cur_msg;

  big_uint16_t data_type;
  memcpy(&data_type, &header[14], sizeof(data_type));
  data_type_ =
      static_cast<uint16_t>(data_type);

  big_uint64_t ntp_time;
  memcpy(&ntp_time, &header[16], sizeof(ntp_time));

  return true;
}

bool Parser::decode_body() {
	ROS_INFO_STREAM("decode body");
  char body[max_body_length] = ""; // allocate and clear data for header
  if (body_length_ >= max_body_length){
	return false;
}
  istream is(&buffer_);
  is.read(body, body_length_);
  //lidar_message_.clear();


  try {
    switch (data_type_) {
    case LidarMessage::SCAN:
      ROS_INFO_STREAM("data type scan");
      process_scan_data(body);
      break;
    case LidarMessage::OBJECT:
      // TODO(roman): debug object data;
      ROS_INFO_STREAM("data type object");
      //process_object_data(body);
      break;
    case LidarMessage::VEHICLE_STATE:
      // TODO(roman): add vehicle state;
      ROS_INFO_STREAM("data type vehicle");
      break;
    case LidarMessage::ERRORS_AND_WARNINGS:
      ROS_INFO_STREAM("data type error");
      process_error_data(body);
      break;
    default:
      ROS_INFO_STREAM("data type unknown");
      LOG(ERROR) << "Unknown Message Type";
    }
  } catch (std::exception const &e) {
    LOG(FATAL) << e.what();
  }
  // Don't hate me for this
  //buffer_.consume(buffer_.size());
  return true;
}

void Parser::process_scan_data(const char *data) {
	ROS_INFO_STREAM("process scan data");
  ScanData *scan = &(lidar_message_.scan_data);
  while (scan->scan_points.size() > 0){
	scan->scan_points.pop_back();
  }
  scan->scan_points.clear();
  little_uint16_t scan_number;
  memcpy(&scan_number, &data[0], sizeof(scan_number));
  scan->scan_number=scan_number;

  little_uint16_t scanner_satus;
  memcpy(&scanner_satus, &data[2], sizeof(scanner_satus));
  process_scanner_status(scanner_satus);

  little_uint16_t sync_phase_offset;
  memcpy(&sync_phase_offset, &data[4], sizeof(sync_phase_offset));
  scan->sync_phase_offset=sync_phase_offset;

  little_uint64_t scan_start_time_ntp;
  memcpy(&scan_start_time_ntp, &data[6], sizeof(scan_start_time_ntp));
  scan->start_time=scan_start_time_ntp;

  little_uint64_t scan_end_time_ntp;
  memcpy(&scan_end_time_ntp, &data[14], sizeof(scan_end_time_ntp));
  scan->end_time=scan_end_time_ntp;

  little_uint16_t angle_ticks_per_rotation;
  memcpy(&angle_ticks_per_rotation, &data[22],
         sizeof(angle_ticks_per_rotation));

  little_int16_t start_angle_ticks;
  memcpy(&start_angle_ticks, &data[24], sizeof(start_angle_ticks));
  scan->start_angle=start_angle_ticks;

  little_int16_t end_angle_ticks;
  memcpy(&end_angle_ticks, &data[26], sizeof(end_angle_ticks));
  scan->end_angle=end_angle_ticks;

  little_uint16_t scan_points;
  memcpy(&scan_points, &data[28], sizeof(scan_points));
  scan->num_scan_points = scan_points;
  // only availabe if vehicle data is available
  // process_mounting_position(&(scan->mounting_position), data + 30);

  // little_uint16_t flags;
  // memcpy(&flags, &data[42], sizeof(flags));

  //// TODO(roman): handle flags
  // if (flags & 1) {
  //// ground labeled
  //} else if (flags & (1 << 1)) {
  //// dirt labeled
  //} else if (flags & (1 << 2)) {
  //// rain labeled
  //}
  
  ROS_INFO_STREAM("get scan points: " << scan_points);
  scan->scan_points.resize(scan_points);
  for (int i = 0; i < scan_points; ++i) {
	try {
      process_scan_point(&(scan->scan_points[i]), &data[44 + 10 * i]);
    } catch (wrong_layer const &e) {
      // the last scan point is not valid - remove it
      scan->scan_points.pop_back();
      LOG(ERROR) << e.what() << " - Message " << i << " will be ignored";
    }
  }
}

void Parser::process_scan_point(ScanPoint *point,
                                const char *data) {
  // fixme(roman): zuweisung für layer passt nicht
  little_uint8_t layer, echo;
  const char low_nibble = (data[0] & 0x0f);
  const char high_nibble = (data[0] & 0xf0);
  memcpy(&layer, &low_nibble, sizeof(layer));
  memcpy(&echo, &high_nibble, sizeof(echo));
  // CHECK_LE(layer, 4);  // catch that
  if (layer >= 4)
  {
    throw wrong_layer(layer);
  }
  point->layer=layer;
  point->echo=echo;
  // little_uint8_t flags;
  // memcpy(&flags, &data[1], sizeof(flags));

  //// todo(roman): handle flags
  // if (flags & 0x01) {
  //// lidar_message_.add_flag();
  //} else if (flags & 0x02) {
  //// lidar_message_.add_flag();
  //} else if (flags & 0x04) {
  //// lidar_message_.add_flag();
  //} else if (flags & 0x08) {
  //// lidar_message_.add_flag();
  //}

  little_int16_t horizontal_angle;
  memcpy(&horizontal_angle, &data[2], sizeof(horizontal_angle));
  point->horizontal_angle=horizontal_angle;
  little_uint16_t radial_distance;
  memcpy(&radial_distance, &data[4], sizeof(radial_distance));
  point->radial_distance=radial_distance;
  little_uint16_t echo_pulse_width;
  memcpy(&echo_pulse_width, &data[6], sizeof(echo_pulse_width));
  point->echo_pule_width=echo_pulse_width;
  ROS_INFO_STREAM("angle tick " << horizontal_angle << " dist " << radial_distance<<"cm");
}

// object data related member functions

void Parser::process_object_data(const char *data) {
  ROS_INFO_STREAM("process object data");
  ObjectData *object_data = &(lidar_message_.object_data);
  little_uint64_t scan_start_timestamp;
  memcpy(&scan_start_timestamp, &data[0], sizeof(scan_start_timestamp));
  object_data->start_time=scan_start_timestamp;

  little_uint16_t number_of_objects;
  memcpy(&number_of_objects, &data[8], sizeof(number_of_objects));

  contour_offset = 0;
  for (int i = 0; i < number_of_objects; ++i) {
    process_object(&(object_data->objects[i]),
                   &data[10 + i * 58 + contour_offset]);
  }
}

void Parser::process_object(Object *object,
                            const char *data) {
  ROS_INFO_STREAM("process object");
  little_uint16_t object_id;
  memcpy(&object_id, &data[0], sizeof(object_id));
  object->id=object_id;

  little_uint16_t object_age;
  memcpy(&object_age, &data[2], sizeof(object_age));
  object->age=object_age;

  little_uint16_t object_prediction_age;
  memcpy(&object_prediction_age, &data[4], sizeof(object_prediction_age));
  object->prediction_age=object_prediction_age;

  little_uint16_t relative_timestamp;
  memcpy(&relative_timestamp, &data[6], sizeof(relative_timestamp));
  object->relative_timestamp=relative_timestamp;

  process_reference_point(&(object->reference_point), &data[8]);
  process_point_2D(&(object->closest_point), &data[16]);
  process_bounding_box(&(object->bounding_box), &data[20]);
  process_object_box(&(object->object_box), &data[28]);

  process_absolute_velocity(&(object->absolute_velocity), &data[38]);
  process_point_2D(&(object->relative_velocity), &data[46]);

  little_uint16_t classification;
  memcpy(&classification, &data[50], sizeof(classification));

  //CHECK_LE(classification, 7) << "Undefined classification value";
  if (classification >= 7){
	throw bad_classification(classification);
  }

    object->classification=static_cast<uint16_t>(classification);
  little_uint16_t certainty;
  memcpy(&certainty, &data[54], sizeof(certainty));
  object->classification_certainty=certainty;

  little_uint16_t number_of_contour_points;
  memcpy(&number_of_contour_points, &data[56],
         sizeof(number_of_contour_points));

  contour_offset += number_of_contour_points * 4;
  for (int i = 0; i < number_of_contour_points; ++i) {
    process_point_2D(&(object->contour_points[i]), &data[58 + 4 * i]);
  }
}

void Parser::process_reference_point(
    ReferencePoint *rf, const char *data) {
  process_point_2D(&(rf->point), data);
  process_point_2D(&(rf->sigma), &data[4]);
}

void Parser::process_bounding_box(
    BoundingBox *bb, const char *data) {
  process_point_2D(&(bb->center), data);
  uint16_t w, l;
  memcpy(&w, &data[4], sizeof(w));
  bb->width=w;
  memcpy(&l, &data[6], sizeof(l));
  bb->length=l;
}

void Parser::process_object_box(ObjectBox *ob,
                                const char *data) {
  process_point_2D(&(ob->center), data);
  process_size_2D(&(ob->size), (data + 4));
  int16_t orientation;
  memcpy(&orientation, &data[8], sizeof(orientation));
  ob->orientation=orientation;
}

void Parser::process_absolute_velocity(
    AbsoluteVelocity *av, const char *data) {
  process_point_2D(&(av->direction), data);
  process_size_2D(&(av->sigma), &data[4]);
}

void Parser::process_point_2D(Point2D *p, const char *data) {
  little_int16_t x, y;
  memcpy(&x, &data[0], sizeof(x));
  p->x=x;

  memcpy(&y, &data[2], sizeof(y));
  p->y=y;
}

void Parser::process_size_2D(Size2D *s, const char *data) {
  little_uint16_t x, y;
  memcpy(&x, &data[0], sizeof(x));
  s->x=x;

  memcpy(&y, &data[2], sizeof(y));
  s->y=y;
}

// ERROR DATA RELATED MEMBER FUNCTIONS

void Parser::process_error_data(const char *data) {
  little_uint16_t error_register_1, error_register_2;
  little_uint16_t warning_register_1, warning_register_2;

  memcpy(&error_register_1, &data[0], sizeof(error_register_1));
  process_error_register_1(error_register_1);

  memcpy(&error_register_2, &data[2], sizeof(error_register_2));
  process_error_register_2(error_register_2);

  memcpy(&warning_register_1, &data[4], sizeof(warning_register_1));
  process_warning_register_1(warning_register_1);

  memcpy(&warning_register_2, &data[6], sizeof(warning_register_2));
  process_warning_register_2(warning_register_2);
}

void Parser::process_scanner_status(little_uint16_t status) {
  // TODO(roman): convert to DLog or VLog
/*  LOG_IF(INFO, status & (1 << 0)) << "motor on";
  LOG_IF(INFO, status & (1 << 1)) << "laser on";
  LOG_IF(INFO, status & (1 << 2)) << "internal feedback";
  LOG_IF(INFO, status & (1 << 3)) << "set frequency reached";
  LOG_IF(INFO, status & (1 << 4)) << "external sync signal detected";
  LOG_IF(INFO, status & (1 << 5)) << "sync ok";
  LOG_IF(INFO, status & (1 << 6)) << "sync master (instead of slave)";
  // 7 reserved
  LOG_IF(INFO, status & (1 << 8)) << "epw compensation on";
  LOG_IF(INFO, status & (1 << 9)) << "system compensation on";
  LOG_IF(INFO, status & (1 << 10)) << "start pulse";
  // 11 - 14 reserved
  LOG_IF(INFO, status & (1 << 15)) << "upside down";*/
}

void Parser::process_error_register_1(little_uint16_t reg) {
  LOG_IF(ERROR, reg & (1 << 0)) << "E-SP : internal error -> contact support";
  LOG_IF(ERROR, reg & (1 << 1))
      << "E-MOTOR_1 : motor fault -> contact support ";
  LOG_IF(ERROR, reg & (1 << 2))
      << "E-BUFFER_1 : scan buffer transmitted incompletly -> decrease scan "
         "resolution/frequency/range; conntact support";
  LOG_IF(ERROR, reg & (1 << 3))
      << "E-BUFFER_1 : scan buffer overflow -> decrease scan "
         "resolution/frequency/range; conntact support";
  // 4-7 reserved
  LOG_IF(ERROR, reg & (1 << 8))
      << "E-TEMP : APD Over Temperature -> provide cooling";
  LOG_IF(ERROR, reg & (1 << 9))
      << "E-TEMP : APD Under Temperature -> provide heating";
  LOG_IF(ERROR, reg & (3 << 8))
      << "E-TEMP : APD Temperature Sensor defect -> contact support";
  LOG_IF(ERROR, reg & (1 << 10))
      << "E-MOTOR_2 : motor fault -> contact support";
  LOG_IF(ERROR, reg & (1 << 11))
      << "E-MOTOR_3 : motor fault -> contact support";
  LOG_IF(ERROR, reg & (1 << 12))
      << "E-MOTOR_4 : motor fault -> contact support";
  LOG_IF(ERROR, reg & (1 << 13))
      << "E-MOTOR_5 : motor fault -> contact support";
}

void Parser::process_error_register_2(little_uint16_t reg) {
  LOG_IF(ERROR, reg & (1 << 0))
      << "E-IF_internal_1 : no scan data received -> contact support";
  LOG_IF(ERROR, reg & (1 << 1))
      << "E-IF_internal_2 : internal communication error -> contact support";
  LOG_IF(ERROR, reg & (1 << 2))
      << "E-IF_internal_2 : incorrect scan data -> contact support";
  LOG_IF(ERROR, reg & (1 << 3))
      << "E-Configuration_1 : FPGA not configurable -> contact support";
  LOG_IF(ERROR, reg & (1 << 4)) << "E-Configuration_2 : incorrect "
                                   "configuration data -> load correct "
                                   "configuration values";
  LOG_IF(ERROR, reg & (1 << 5)) << "E-Configuration_3 : configuration "
                                   "contains incorrect parameters->load "
                                   "correct configuration values ";
  LOG_IF(ERROR, reg & (1 << 6)) << "E-Timeout_1 : data processing timeout -> "
                                   "decrease scan resolution or scan "
                                   "frequency";
  LOG_IF(ERROR, reg & (1 << 7)) << "E-Timeout_2 : reset the computation of "
                                   "the environmental model -> contact "
                                   "support ";
}

void Parser::process_warning_register_1(little_uint16_t reg) {
  LOG_IF(WARNING, reg & (1 << 0)) << "W-CMD : internal communication error";
  LOG_IF(WARNING, reg & (1 << 3)) << "W-low_temperature : temperature to low "
                                     "-> warning of insufficient temperature";
  LOG_IF(WARNING, reg & (1 << 4)) << "W-high_temperature : temperature to "
                                     "high -> warning of exceeding "
                                     "temperature";
  LOG_IF(WARNING, reg & (1 << 5)) << "W-Motor_1 : internal warning";
  LOG_IF(WARNING, reg & (1 << 7)) << "W-Sync : synchronisation error -> "
                                     "check synchronisation- and scan "
                                     "frequently";
  LOG_IF(WARNING, reg & (1 << 12)) << "W-SP_1 : start pulse missing (laser 1)";
  LOG_IF(WARNING, reg & (1 << 13)) << "W-SP_2 : start pulse missing (laser 2)";
}

void Parser::process_warning_register_2(little_uint16_t reg) {
  LOG_IF(WARNING, reg & (1 << 0)) << "W-IF_CAN : CAN interface blocked -> "
                                     "check CAN bus and CAN connection";
  LOG_IF(WARNING, reg & (1 << 1))
      << "W-IF_ETH : Ethernet interface blocked -> check Ethernet connection";
  LOG_IF(WARNING, reg & (1 << 2))
      << "W-CANdata : incorrect CAN message received -> check CAN data";
  LOG_IF(WARNING, reg & (1 << 3))
      << "W-IF_internal_1 : incorrect scan data -> contact support";
  LOG_IF(WARNING, reg & (1 << 4))
      << "W-ETHdata : unknown or incorrect data -> check Ethernet data";
  LOG_IF(WARNING, reg & (1 << 5)) << "W-Command : incorrect or forbidden "
                                     "command received -> check command";
  LOG_IF(WARNING, reg & (1 << 6)) << "W-Flash : memory access failure -> "
                                     "restart ibeo LUX, conntact support";
  LOG_IF(WARNING, reg & (1 << 7))
      << "W-Overflow_1 : internal overflow -> contact support";
  LOG_IF(WARNING, reg & (1 << 8)) << "W-EgoMotion : vehicle data update "
                                     "missing -> check CAN vehicle data";
  LOG_IF(WARNING, reg & (1 << 9)) << "W-MountingPosition : incorrect "
                                     "mounting parameters -> correct "
                                     "mounting position according to OM";
  LOG_IF(WARNING, reg & (1 << 10))
      << "W-CalcFrequency : no object computation due to scan frequency->set "
         "the scan frequency to 12.5 Hz to receive objects ";
}

void Parser::process_mounting_position(
    MountingPosition *m_pos, const char *data) {
  little_int16_t yaw, pitch, roll, x, y, z;
  memcpy(&yaw, &data[0], sizeof(yaw));
  m_pos->yaw=yaw;

  memcpy(&pitch, &data[2], sizeof(pitch));
  m_pos->pitch=pitch;

  memcpy(&roll, &data[4], sizeof(roll));
  m_pos->roll=roll;

  memcpy(&x, &data[6], sizeof(x));
  m_pos->x=x;

  memcpy(&y, &data[8], sizeof(y));
  m_pos->y=y;

  memcpy(&z, &data[10], sizeof(z));
  m_pos->z=z;
  DLOG(INFO) << "Mounting Postion: ";
  DLOG(INFO) << "  yaw:   " << yaw;
  DLOG(INFO) << "  pitch: " << pitch;
  DLOG(INFO) << "  roll:  " << roll;
  DLOG(INFO) << "  x:     " << x;
  DLOG(INFO) << "  y:     " << y;
  DLOG(INFO) << "  z:     " << z;
}

LidarMessage &Parser::get_message() { return lidar_message_; }

