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

#include "gnss/R9300/parser/R9300_ascii_parser.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {
double str2Seconds(const char *timeStr) {
  time_t tt =
      std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  struct tm *ptm = localtime(&tt);

  std::tm t = *ptm;
  sscanf(timeStr, "%2d%2d%2d", &t.tm_hour, &t.tm_min, &t.tm_sec);
  double decimal = 0;
  sscanf(timeStr, "%lf", &decimal);
  int milliseconds = decimal;
  decimal -= milliseconds;
  double ret = std::mktime(&t);
  ret += decimal;
  return ret;
}

enum LocalMsgType {
  GNGGA = 0,     // $GPGGA
  GNVTG = 1,     // $GPVTG
  GNZDA = 2,     // GNZDA
  BestPos = 3,   // #BESTPOSA
  Heading2 = 4,  // #HEADING2A
  RawImu = 5,    // #RAWIMUA
  LocalMsgTypeCount = 6
};

const struct {
  const char *name;
  std::size_t len;
} LocalMsgNames[] = {{"$GNGGA", 6}, {"$GNVTG", 6}, {"$GNZDA", 6}};

LocalMsgType FindLocalMsgType(const char *buf) {
  LocalMsgType ret = LocalMsgTypeCount;
  int beg = 0, end = 0;
  if (*buf == '$') {
    beg = 0;
    end = LocalMsgType::BestPos;
  }

  for (; beg < end; ++beg) {
    if (!memcmp(buf, LocalMsgNames[beg].name, LocalMsgNames[beg].len)) {
      ret = static_cast<LocalMsgType>(beg);
      break;
    }
  }
  return ret;
}
enum DecimalType { TenDecimal = 0, OtcDecimal, HexDecimal };

template <typename T>
T Str2Type(const char *buf, DecimalType decimalType = TenDecimal) {
  T d = 0;
  std::istringstream strStream(buf);
  switch (decimalType) {
    case DecimalType::HexDecimal:
      strStream >> std::hex >> d;
      break;
    case DecimalType::OtcDecimal:
      strStream >> std::oct >> d;
      break;
    case DecimalType::TenDecimal:
    default:
      strStream >> d;
      break;
  }

  return d;
}

template <>
std::string Str2Type<std::string>(const char *buf, DecimalType) {
  std::string ret;
  while (*buf != ',' && *buf != '*' && *buf != ';') {
    ret.push_back(*buf);
    ++buf;
  }
  return ret;
}

char Move2Next(const char **beg) {
  const char *ptr = *beg;
  while (*ptr != ',' && *ptr != '*' && *ptr != ';') {
    ++ptr;
  }
  char ret = *ptr++;
  *beg = ptr;
  return ret;
}

float GpsToDecimalDegrees(const char *nmeaPos, char quadrant) {
  float v = 0;
  if (strlen(nmeaPos) > 5) {
    char integerPart[3 + 1];
    int digitCount = (nmeaPos[4] == '.' ? 2 : 3);
    memcpy(integerPart, nmeaPos, digitCount);
    integerPart[digitCount] = 0;
    nmeaPos += digitCount;
    v = atoi(integerPart) + atof(nmeaPos) / 60.;
    if (quadrant == 'W' || quadrant == 'S') v = -v;
  }
  return v;
}

}  // namespace
namespace civ {
namespace drivers {
namespace R9300 {

using civ::drivers::gnss::GNSSmsgType;
using civ::drivers::gnss::spGNRMC;

R9300AsciiParser::R9300AsciiParser() {
  std::string outputName = "result.txt";
  fileFd_ = open(outputName.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);

  if (fileFd_ == -1) {
    std::cout << "open output file failed " << outputName << std::endl;
    fileFd_ = open(outputName.c_str(), O_RDWR | O_CREAT, 0755);
    if (fileFd_ > 0) {
      std::cout << "created file " << outputName << std::endl;
    }
  }
}

R9300AsciiParser::~R9300AsciiParser() {
  close(fileFd_);
  fileFd_ = -1;
}
void R9300AsciiParser::ParseData(std::vector<GNSSmsg> &v_gnss) {
  auto end = this->data_end_;
  auto beg = this->data_;
  int data_len = this->data_end_ - this->data_;
  std::size_t shift = 0;

  gnss::GNSSmsg gnss;
  // std::vector<gnss::GNSSmsg> v_gnss;
  while (ParseGnssStr(reinterpret_cast<const char *>(beg), data_len, shift,
                      &gnss)) {
    auto msgType =
        FindLocalMsgType(reinterpret_cast<const char *>(gnss.buffer));

    switch (msgType) {
      case LocalMsgType::GNGGA: {
        gnss.type_ = GNSSmsgType::GNGGA;
      } break;
      case LocalMsgType::GNVTG: {
        gnss.type_ = GNSSmsgType::GNVTG;
      } break;
      case LocalMsgType::GNZDA: {
        gnss.type_ = GNSSmsgType::GNZDA;
      } break;

      default:
        gnss.type_ = GNSSmsgType::None;
        break;
    }

    v_gnss.push_back(gnss);
    auto ptr = beg + shift;
    if (ptr > end) {
      this->data_ = this->data_end_;
      break;
    } else {
      this->data_ = ptr;
      beg = this->data_;
      data_len -= shift;
    }
  }

  for (const auto &gnss_msg : v_gnss) {
    if (gnss_msg.type_ == GNSSmsgType::GNGGA) {
      auto msg = std::make_shared<civ::drivers::rawgnss::GNGGA>();
      ParseGNGGA(gnss_msg, msg);
      gngga_writer_->Write(msg);
    }
    if (gnss_msg.type_ == GNSSmsgType::GNVTG) {
      auto msg = std::make_shared<civ::drivers::rawgnss::GNVTG>();
      ParseGNVTG(gnss_msg, msg);
      gnvtg_writer_->Write(msg);
    }
  }
}

bool R9300AsciiParser::InitWriters(std::unique_ptr<apollo::cyber::Node> &node) {
  if (node == nullptr) {
    return false;
  }
  gngga_writer_ = node->CreateWriter<civ::drivers::rawgnss::GNGGA>("/GNGGA");
  if (gngga_writer_ == nullptr) {
    return false;
  }

  gnvtg_writer_ = node->CreateWriter<civ::drivers::rawgnss::GNVTG>("/GNVTG");
  if (gnvtg_writer_ == nullptr) {
    return false;
  }
  return true;
}

void output_buff(char *buff, int size) {
  for (int i = 0; i < size; i++) {
    printf("%c", buff[i]);
  }
  printf("\n");
}
void R9300AsciiParser::ParseGNGGA(
    const civ::drivers::gnss::GNSSmsg &gnss_msg,
    std::shared_ptr<civ::drivers::rawgnss::GNGGA> msg) {
  std::vector<char *> res_split;
  reinterpret_cast<const char *>(gnss_msg.buffer);

  char buff_tmp[200];
  memcpy(buff_tmp, gnss_msg.buffer, gnss_msg.buffer_len);

  output_buff(buff_tmp, gnss_msg.buffer_len);
  // strcpy(buff_tmp, reinterpret_cast<const char*>(gnss_msg.buffer));
  const char split[] = ",";
  char *res = std::strtok(buff_tmp, split);

  while (res != NULL) {
    res_split.push_back(res);
    res = strtok(NULL, split);
  }
  // auto msg = std::make_shared<civ::drivers::rawgnss::GNGGA>();
  if (res_split.size() > 12) {
    msg->set_utc_hhmmss(res_split[1]);
    msg->set_utc_in_seconds(str2Seconds(res_split[1]));
    msg->set_latitude(std::stof(res_split[2]));
    msg->set_latitude_dir(civ::drivers::rawgnss::GNGGA::North);
    msg->set_longitude(std::stof(res_split[4]));
    msg->set_longitude_dir(civ::drivers::rawgnss::GNGGA::East);
    msg->set_quality(std::stoi(res_split[6]));
    msg->set_num_of_satellites_used(std::stoi(res_split[7]));
    msg->set_hdop(std::stof(res_split[8]));
    msg->set_alt(std::stof(res_split[9]));

    float latitude_deg = GpsToDecimalDegrees(res_split[2], 'N');
    float longitude_deg = GpsToDecimalDegrees(res_split[4], 'E');
    msg->set_latitude_deg(latitude_deg);
    msg->set_longitude_deg(longitude_deg);
  }
}

void R9300AsciiParser::ParseGNVTG(
    const civ::drivers::gnss::GNSSmsg &gnss_msg,
    std::shared_ptr<civ::drivers::rawgnss::GNVTG> msg) {
  std::vector<char *> res_split;
  reinterpret_cast<const char *>(gnss_msg.buffer);

  char buff_tmp[200];
  memcpy(buff_tmp, gnss_msg.buffer, gnss_msg.buffer_len);

  output_buff(buff_tmp, gnss_msg.buffer_len);
  // strcpy(buff_tmp, reinterpret_cast<const char*>(gnss_msg.buffer));
  const char split[] = ",";
  char *res = std::strtok(buff_tmp, split);

  while (res != NULL) {
    res_split.push_back(res);
    res = strtok(NULL, split);
  }
  // auto msg = std::make_shared<civ::drivers::rawgnss::GNVTG>();
  if (res_split.size() > 5) {
    msg->set_track_true(std::stof(res_split[1]));
    msg->set_track_mag(std::stof(res_split[3]));
    msg->set_speed_kn(std::stof(res_split[5]));
    msg->set_speed_km(std::stof(res_split[7]));
  }
}

bool R9300AsciiParser::ParseGnssStr(const char *buff, const uint32_t &buff_len,
                                    std::size_t &shift, GNSSmsg *gnss) {
  const char *raw_gnss = strstr(buff, "$");
  if (raw_gnss) {
    const char *p = strchr(raw_gnss, '*');
    if (p) {
      p += 5;
      if ((p - reinterpret_cast<const char *>(buff)) <= buff_len) {
        gnss->buffer_len = static_cast<uint32_t>(p - raw_gnss);
        memcpy(reinterpret_cast<char *>(gnss->buffer), raw_gnss,
               static_cast<size_t>(gnss->buffer_len));
        shift = p - buff;
        return true;
      } else {
        return false;
      }
    } else {
      return false;
    }
  }
  return false;
}

// bool R9300AsciiParser::ParseGnrmcStr(const char *buff, const uint32_t
// &buff_len,
//                                      std::size_t &shift, gnss::GNRMC *gnrmc)
//                                      {
//   const char *raw_gnrmc = strstr(buff, "$GNRMC");
//   if (raw_gnrmc) {
//     const char *p = strchr(raw_gnrmc, '*');
//     if (p) {
//       p += 5;
//       if ((p - reinterpret_cast<const char *>(buff)) <= buff_len) {
//         gnrmc->buffer_len = static_cast<uint32_t>(p - raw_gnrmc);
//         memcpy(reinterpret_cast<char *>(gnrmc->buffer), raw_gnrmc,
//                static_cast<size_t>(gnrmc->buffer_len));
//         shift = p - buff;
//         return true;
//       } else {
//         return false;
//       }
//     } else {
//       return false;
//     }
//   }
//   return false;
// }
}  // namespace R9300
}  // namespace drivers
}  // namespace civ
