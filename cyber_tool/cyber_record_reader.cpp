/******************************************************************************
 * Copyright 2018 The CIV-AI Authors. All Rights Reserved.
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

#include "cyber_record_reader.hpp"

#include "cyber/cyber.h"
#include "cyber/record/record_reader.h"
//
#include "common/util/time_system.hpp"
#include "common/util/util.h"
namespace civ {
namespace cyber_tool {

using apollo::cyber::record::RecordReader;

CyberRecordReader::CyberRecordReader() {}

CyberRecordReader::~CyberRecordReader() {}

//
void CyberRecordReader::Play(const std::string &file_name) {
  if (file_name == "") {
    AERROR << "file_name is empty";
    return;
  }
  if (file_name.find('*') == std::string::npos) {
    PlaySingle(file_name);
  } else {
    auto const &files = civ::common::util::ListSameNameFile(file_name);
    for (auto const &name : files) {
      AINFO << name;
    }
    for (auto const &name : files) {
      PlaySingle(name);
    }
  }
}

// void CyberRecordReader::Subscribe(
//     const std::string &topic,
//     const std::function<void(const std::string &)> call_back) {
//   call_back_map_[topic] = call_back;
//   topics_.push_back(topic);
// }

void CyberRecordReader::PlaySingle(const std::string &file_name) {
  AINFO << " File_name: " << file_name;
  RecordReader reader(file_name);
  apollo::cyber::record::RecordMessage message;
  while (apollo::cyber::OK() && reader.ReadMessage(&message)) {
    // 时间过滤
    // if ((message.time < 1627889655.0 * 1e9) || (message.time > 1627890540.2 *
    // 1e9)) {
    //   continue;
    // }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    auto itr = call_back_map_.find(message.channel_name);
    if (itr != call_back_map_.end()) {
      itr->second(message.content);
    }
  }
}

}  // namespace cyber_tool
}  // namespace civ
