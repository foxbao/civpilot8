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

#include "imu/wheeltecN100/wheeltec_component.h"
#include <string>
#include "driver/common/util.hpp"
#include "imu/wheeltecN100/parser/wheeltec_binary_parser.h"

namespace civ {
namespace drivers {
namespace wheeltec {

using civ::drivers::wheeltec::WheeltecBinaryParser;
WheeltecComponent::WheeltecComponent() {}
WheeltecComponent::~WheeltecComponent() {
  if (serial_) {
    delete serial_;
  }
}
void WheeltecComponent::SetCallBack(
    const std::function<void(const char *res)> &cb) {
  m_callback = cb;
}
bool WheeltecComponent::Init() {return true;}

int WheeltecComponent::Read(const char *device_name, uint32_t baud_rate) {
  std::cout << "connecting to " << device_name << ":" << baud_rate << std::endl;
  WheeltecBinaryParser parser;
  int remainSize = 0;
  int buff_size = 2048;
  unsigned char buff[buff_size];
  std::string dataBuffer;
  dataBuffer.resize(buff_size);

  serial_ = SerialStream::CreateSerial(device_name, baud_rate);

  if (serial_ && serial_->Connect()) {
    apollo::cyber::Init("wheeltec");
    
    auto node = apollo::cyber::CreateNode("IMU");
    if (!parser.InitWriters(node)) {
      return -1;
    }
    auto bufferPtr = reinterpret_cast<unsigned char *>(
        const_cast<char *>(dataBuffer.data()));
    int count = 0;

    using STATUS = civ::drivers::common::Stream::Status;
    while (serial_->get_status() == STATUS::CONNECTED) {
      auto length =
          serial_->read(bufferPtr + remainSize, buff_size - remainSize);
      if (length) {
        length += remainSize;
        // memset(current_serial_data_, 0, serial_data_size_);
        // memcpy(current_serial_data_, bufferPtr, length);
        current_data_hex_ = civ::drivers::binaryToHex(bufferPtr, length);
        if (m_callback) {
          m_callback(current_data_hex_.c_str());  // 该函数指针可能是个野指针
        }

        parser.UpdateDataPtr(bufferPtr, length);
        parser.ParseData();
        remainSize = parser.RemainSize();
        if (remainSize < length && remainSize > 0) {
          memcpy(bufferPtr, bufferPtr + length - remainSize, remainSize);
        }
      }
      count++;
      if (count % 100 == 0) {
        std::cout << "reading imu from port" << std::endl;
        count = 0;
      }
    }

  } else {
    std::cout << "cannot connect" << std::endl;
    return -1;
  }

  return 0;
}

void WheeltecComponent::Stop() {
  if (serial_) {
    serial_->Disconnect();
  }
}


}  // namespace wheeltec
}  // namespace drivers
}  // namespace civ
