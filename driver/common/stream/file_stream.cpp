/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the CIV License, Version 2.0 (the "License");
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

#include "common/stream/file_stream.hpp"
#include "common/interface/logger/log_interface.h"
namespace civ {
namespace drivers {
namespace common {

FileStream::FileStream(std::string const &file_path) : file_path_(file_path) {
  uptr_stream_.reset(new std::fstream());
}

FileStream *FileStream::CreateFile(std::string const &path) {
  return path == "" ? nullptr : new FileStream(path);
}

FileStream::~FileStream() { this->Disconnect(); }

bool FileStream::Connect(bool is_read) {
  if (!uptr_stream_->is_open()) {
    if (is_read) {
      uptr_stream_->open(file_path_, std::fstream::in | std::fstream::binary);
    } else {
      uptr_stream_->open(file_path_, std::fstream::out | std::fstream::binary);
    }

    if (!uptr_stream_->is_open()) {
      AERROR << "open " << file_path_ << " failed";
      return false;
    }
    AINFO << "open " << file_path_ << " stream ok!";
  }
  return true;
}

bool FileStream::Disconnect() {
  if (uptr_stream_) {
    uptr_stream_->close();
  }
  return true;
}

size_t FileStream::read(uint8_t *buffer, size_t max_length) {
  uptr_stream_->read(reinterpret_cast<char *>(buffer), max_length);
  if (!*uptr_stream_) {
    AINFO << file_path_ << " read ended. ";
  }
  return uptr_stream_->gcount();
}

void FileStream::write(const uint8_t *data, size_t length) {
  if (!uptr_stream_->is_open()) {
    if (!Connect()) {
      return;
    }
    AERROR << "Connect " << file_path_ << " success.";
  }
  uptr_stream_->write(reinterpret_cast<const char *>(data), length);
}
}  // namespace common
}  // namespace drivers
}  // namespace civ
