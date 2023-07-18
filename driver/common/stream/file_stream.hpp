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

#include <fstream>
#include <memory>
#include <string>

namespace civ {
namespace drivers {
namespace common {
class FileStream final {
 public:
  FileStream(std::string const &file_path);
  static FileStream *CreateFile(std::string const &path);
  ~FileStream();

  bool Connect(bool is_read = true);
  bool Disconnect();

  size_t read(uint8_t *buffer, size_t max_length);
  void write(const uint8_t *data, size_t length);

 private:
  std::string const &file_path_ = "";
  std::unique_ptr<std::fstream> uptr_stream_ = nullptr;
};
}  // namespace common
}  // namespace drivers
}  // namespace civ
