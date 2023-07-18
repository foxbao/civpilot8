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
#pragma once
#include <memory>
#include "common/parser/parser.h"
#include "cyber/cyber.h"
#include "cyber/node/node.h"
#include "gnss/R9300/messages/R9300_messages.h"
#include "message/drivers/gnss/proto/raw_gnss.pb.h"
namespace civ {
namespace drivers {
namespace R9300 {
using civ::drivers::common::Parser;
using civ::drivers::gnss::GNGGA;
using civ::drivers::gnss::GNRMC;

using civ::drivers::gnss::GNSSmsg;
class R9300AsciiParser : public Parser {
 public:
  R9300AsciiParser();
  ~R9300AsciiParser();
  // size of data left in the buffer which are not yet parsed
  void ParseData(std::vector<GNSSmsg> &v_gnss);
  std::size_t RemainSize() { return std::distance(data_, data_end_); }
  bool InitWriters(std::unique_ptr<apollo::cyber::Node> &node);

 public:
  /**
   * @brief extract one line of gnss data which starts with $, like
   * $GNRMC,,V,,,,,,,,,,N*4D   OR
   * $GNRMC,,V,,,,,,,,,,N*4D
   *
   * @param buff current buffer containing all the data
   * @param buff_len
   * @param shift the new begin of buff after parsing
   * @return gnss the extracted msg
   */
  bool ParseGnssStr(const char *buff, const uint32_t &buff_len,
                    std::size_t &shift, GNSSmsg *gnss);
  void ParseGNGGA(const civ::drivers::gnss::GNSSmsg &gnss_msg,
                  std::shared_ptr<civ::drivers::rawgnss::GNGGA> msg);

  void ParseGNVTG(const civ::drivers::gnss::GNSSmsg &gnss_msg,
                  std::shared_ptr<civ::drivers::rawgnss::GNVTG> msg);

//   void PublishLocation(const civ::drivers::gnss::GNSSmsg &gnss_msg);

 private:
  int fileFd_;
  std::string result_file_name_ = "result.txt";
  std::shared_ptr<apollo::cyber::Writer<civ::drivers::rawgnss::GNGGA>>
      gngga_writer_;
  std::shared_ptr<apollo::cyber::Writer<civ::drivers::rawgnss::GNVTG>>
      gnvtg_writer_;
};
}  // namespace R9300
}  // namespace drivers
}  // namespace civ
