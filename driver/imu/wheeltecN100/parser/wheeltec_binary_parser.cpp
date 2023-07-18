/******************************************************************************
 * Copyright 2022 The CIV Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "imu/wheeltecN100/parser/wheeltec_binary_parser.h"
#include "common/wheeltec/wheeltec_gflags.h"
#include <cstring>
#include <iostream>
#include <memory>
#include "imu/wheeltecN100/Converter.hpp"
// #include "common/serial_port/Converter.hpp"
using apollo::cyber::Time;
namespace civ {
namespace drivers {
namespace wheeltec {

std::string binaryToHex(const uint8_t *data, int length) {
  std::string ret;
  static const char *hex = "0123456789ABCDEF";
  for (int i = 0; i < length; i++) {
    ret.push_back(hex[(data[i] >> 4) & 0xf]);  // 取二进制高四位
    ret.push_back(hex[data[i] & 0xf]);         // 取二进制低四位
  }
  return ret;
}


using civ::drivers::imu::CorrectedImu;
using civ::drivers::imu::IMUData_Packet_t;

WheeltecBinaryParser::WheeltecBinaryParser() {}

WheeltecBinaryParser::~WheeltecBinaryParser() {}

bool WheeltecBinaryParser::InitWriters(
    std::unique_ptr<apollo::cyber::Node> &node) {
  if (node == nullptr) {
    return false;
  }
  FLAGS_raw_wheeltec_rawimu;
  // name is set in /common/wheeltec/wheeltec_gflags.h
  imu_writer_ = node->CreateWriter<CorrectedImu>(FLAGS_raw_wheeltec_rawimu);

  // imu_writer_ = node->CreateWriter<CorrectedImu>("/WHEELTEC");
  if (imu_writer_ == nullptr) {
    return false;
  }
  return true;
}
void WheeltecBinaryParser::ParseData(void) {
  auto end = this->data_end_;
  // auto headerEnd = end;
  auto beg = this->data_;

  int data_len = this->data_end_ - this->data_;

  std::size_t shift = 0;

  spIMUData_Packet_t imu = std::make_shared<IMUData_Packet_t>();

  ParseData(beg, data_len, &shift, imu);

  this->data_ = this->data_end_;
}


bool WheeltecBinaryParser::ParseData(const uint8_t *buff,
                                     const uint32_t &buff_len,
                                     std::size_t *shift,
                                     spIMUData_Packet_t imumsg) {
  // std::string data_to_parse = binaryToHex(buff, buff_len);

  if (buff[0] == FRAME_HEAD && buff[1] == TYPE_IMU && buff[2] == IMU_LEN) {
    imumsg->gyroscope_x =
        civ::civloc::DATA_Trans(buff[7], buff[8], buff[9], buff[10]);
    imumsg->gyroscope_y =
        civ::civloc::DATA_Trans(buff[11], buff[12], buff[13], buff[14]);
    imumsg->gyroscope_z =
        civ::civloc::DATA_Trans(buff[15], buff[16], buff[17], buff[18]);

    imumsg->accelerometer_x =
        civ::civloc::DATA_Trans(buff[19], buff[20], buff[21], buff[22]);  // acc
    imumsg->accelerometer_y =
        civ::civloc::DATA_Trans(buff[23], buff[24], buff[25], buff[26]);
    imumsg->accelerometer_z =
        civ::civloc::DATA_Trans(buff[27], buff[28], buff[29], buff[30]);
    imumsg->magnetometer_x =
        civ::civloc::DATA_Trans(buff[31], buff[32], buff[33], buff[34]);  // mag
    imumsg->magnetometer_y =
        civ::civloc::DATA_Trans(buff[35], buff[36], buff[37], buff[38]);
    imumsg->magnetometer_z =
        civ::civloc::DATA_Trans(buff[39], buff[40], buff[41], buff[42]);
    imumsg->Timestamp =
        civ::civloc::timestamp(buff[55], buff[56], buff[57], buff[58]);  // time

    double time = apollo::cyber::Time::Now().ToSecond();
    PublishIMU(imumsg, time);
    // IMUData2PC(imumsg);
  }
  return false;
}

void WheeltecBinaryParser::PublishIMU(sp_cIMUData_Packet_t imumsg,
                                      const double &timestamp) {
  auto msg = std::make_shared<CorrectedImu>();
  msg->set_measurement_time(timestamp);
  civ::common::Point3D *linear_acceleration =
      msg->mutable_linear_acceleration();
  linear_acceleration->set_x(imumsg->accelerometer_x);
  linear_acceleration->set_y(imumsg->accelerometer_y);
  linear_acceleration->set_z(imumsg->accelerometer_z);
  civ::common::Point3D *angular_velocity = msg->mutable_angular_velocity();
  angular_velocity->set_x(imumsg->gyroscope_x);
  angular_velocity->set_y(imumsg->gyroscope_y);
  angular_velocity->set_z(imumsg->gyroscope_z);

  imu_writer_->Write(msg);
}
void WheeltecBinaryParser::IMUData2PC(spIMUData_Packet_t imuMsg) {
  printf("IMU:The Timestamp =  %d\r\n", imuMsg->Timestamp);
}
}  // namespace wheeltec
}  // namespace drivers
}  // namespace civ
