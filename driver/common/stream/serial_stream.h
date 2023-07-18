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

#include <termios.h>
#include <string>
#include "common/stream/stream.h"

namespace civ {
namespace drivers {
namespace common {

class SerialStream : public Stream {
 public:
  static SerialStream* CreateSerial(const char* device_name, uint32_t baud_rate,
                                    uint32_t timeout_usec = 0);
  ~SerialStream();

  bool Connect();
  bool Disconnect();
  size_t read(uint8_t* buffer, size_t max_length);
  size_t write(const uint8_t* data, size_t length);

 private:
  explicit SerialStream(const char* device_name, speed_t baud_rate,
                        uint32_t timeout_usec);
  SerialStream() = delete;
  SerialStream(const SerialStream&) = delete;
  SerialStream(SerialStream&&) = delete;

  void open();
  void close();
  bool configure_port(int fd);
  bool wait_readable(uint32_t timeout_us);
  bool wait_writable(uint32_t timeout_us);
  void check_remove();

  std::string device_name_;
  speed_t baud_rate_;
  uint32_t bytesize_;
  uint32_t parity_;
  uint32_t stopbits_;
  uint32_t flowcontrol_;
  uint32_t byte_time_us_;

  uint32_t timeout_usec_;
  int fd_{-1};
  int errno_;
  bool is_open_;
};
}  // namespace common
}  // namespace drivers
}  // namespace civ
