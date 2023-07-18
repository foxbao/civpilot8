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

#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace civ {
namespace cyber_tool {

/**
 * @class CyberRecordReader
 * @brief Read messages from cyber record.
 */
class CyberRecordReader {
 public:
  CyberRecordReader();
  ~CyberRecordReader();

  void Subscribe(const std::string& topic,
                 const std::function<void(const std::string&)> call_back);
  // 1. 如果出现*号则运行所有同名文件
  // 2. 没有*号则为单独文件
  void Play(const std::string& file_name);

  // std::unordered_map<std::string, std::function<void(const std::string&)>>&
  // Subscribe(std::string const name) {
  //   return call_back_map_;
  // }
  std::function<void(const std::string&)>& operator[](const std::string& name) {
    return call_back_map_[name];
  }

 private:
  void PlaySingle(const std::string& file_name);
  std::vector<std::string> topics_;
  std::unordered_map<std::string, std::function<void(const std::string&)>>
      call_back_map_;
};

}  // namespace cyber_tool
}  // namespace civ
