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
#include <string>
#include "common/stream/serial_stream.h"
#include "common/gnss_third_party/rtklib.h"
#include "common/qianxun/include/qxwz_sdk.h"

#define DEMO_LOG(fmt, ...) printf("[DEMO]" fmt, ##__VA_ARGS__)

extern std::string g_gga_upload;

using civ::drivers::common::SerialStream;
class Nosr {
 public:
  Nosr();
  ~Nosr();
  void sdk_test(std::string device_name, int baud);
  static qxwz_void_t demo_show_caps(qxwz_sdk_cap_info_t *cap_info);
  static qxwz_void_t demo_on_auth(qxwz_int32_t status_code,
                                  qxwz_sdk_cap_info_t *cap_info);
  static qxwz_void_t demo_on_start(qxwz_int32_t status_code,
                                   qxwz_uint32_t cap_id);
  static qxwz_void_t demo_on_status(int code);
  static qxwz_void_t demo_on_data(qxwz_sdk_data_type_e type,
                                  const qxwz_void_t *data, qxwz_uint32_t len);

 private:
  static SerialStream *serial_;

 private:
  static qxwz_uint32_t glob_flag;
  static qxwz_uint64_t glob_ts;
  static qxwz_uint32_t sdk_auth_flag;
  static qxwz_uint32_t sdk_start_flag;
  static qxwz_sdk_cap_info_t sdk_cap_info;
};
