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

#include "nosr.h"
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include "common/stream/serial_stream.h"

// #define DEMO_GGA_STR
// "$GPGGA,000001,3112.518576,N,12127.901251,E,1,8,1,0,M,-32,M,3,0*4B"
#define DEMO_GGA_STR \
  "$GNGGA,102156.00,3117.01616,N,12109.62582,E,1,10,1.02,19.8,M,9.1,M,,*4D"
std::string g_gga_upload = "in nosr.cpp";

typedef __uint8_t uint8_t;

using civ::drivers::common::SerialStream;

// static qxwz_uint32_t glob_flag = 0;
// static qxwz_uint64_t glob_ts = 0;

// static qxwz_uint32_t sdk_auth_flag = 0;
// static qxwz_uint32_t sdk_start_flag = 0;
// static qxwz_sdk_cap_info_t sdk_cap_info = {0};

qxwz_uint32_t Nosr::glob_flag = 1;
qxwz_uint64_t Nosr::glob_ts = 1;

qxwz_uint32_t Nosr::sdk_auth_flag = 1;
qxwz_uint32_t Nosr::sdk_start_flag = 1;
qxwz_sdk_cap_info_t Nosr::sdk_cap_info = qxwz_sdk_cap_info_t();
SerialStream *Nosr::serial_ = nullptr;

Nosr::Nosr() {}

Nosr::~Nosr() {}

qxwz_void_t Nosr::demo_on_status(int code) {
  DEMO_LOG(" on status code: %d\n", code);
}

qxwz_void_t Nosr::demo_show_caps(qxwz_sdk_cap_info_t *cap_info) {
  qxwz_int32_t loop = 0;

  DEMO_LOG("total capabilities: %d\n", cap_info->caps_num);
  for (loop = 0; loop < cap_info->caps_num; ++loop) {
    DEMO_LOG(
        "idx: %d, cap_id: %u, state: %d, act_method: %d, expire_time: %llu\n",
        loop + 1, cap_info->caps[loop].cap_id, cap_info->caps[loop].state,
        cap_info->caps[loop].act_method, cap_info->caps[loop].expire_time);
  }
}

qxwz_void_t Nosr::demo_on_auth(qxwz_int32_t status_code,
                               qxwz_sdk_cap_info_t *cap_info) {
  if (status_code == QXWZ_SDK_STAT_AUTH_SUCC) {
    sdk_auth_flag = 1;
    sdk_cap_info = *cap_info;
    demo_show_caps(cap_info);
  } else {
    DEMO_LOG("auth failed, code=%d\n", status_code);
  }
}

qxwz_void_t Nosr::demo_on_start(qxwz_int32_t status_code,
                                qxwz_uint32_t cap_id) {
  DEMO_LOG("on start cap:status_code=%d, cap_id=%d\n", status_code, cap_id);
  sdk_start_flag = 1;
}

qxwz_void_t Nosr::demo_on_data(qxwz_sdk_data_type_e type,
                               const qxwz_void_t *data, qxwz_uint32_t len) {
  rtcm_t rtcm_;
  init_rtcm(&rtcm_);
  // uint
  DEMO_LOG(" on data: %d, ptr: %p, len: %d\n", type, data, len);

  unsigned char *ptr_data_chat = (unsigned char *)data;
  char *rtcm_data = (char *)data;
  serial_->write(ptr_data_chat, len);

  int status;
  switch (type) {
    case QXWZ_SDK_DATA_TYPE_RAW_NOSR:
      DEMO_LOG("QXWZ_SDK_DATA_TYPE_RAW_NOSR\n");

      for (int i = 0; i < len; i++) {
        status = input_rtcm3(&rtcm_, ptr_data_chat[i]);
        switch (status) {
          case 1:
            break;
        }
      }
      //
      //
      break;
    default:
      DEMO_LOG("unknown type: %d\n", type);
  }
}

void trace(const char *str, int str_size) {
  printf("Trace: \n");
  // write(1, str, str_size);
  std::cout << str << std::endl;
  printf("\n");
}
void error(const char *str, int str_size) {
  printf("Error: ");
  // write(1, str, str_size);
  std::cout << str << std::endl;
  printf("\n");
}

void output_buff(char *buff, int size) {
  for (int i = 0; i < size; i++) {
    printf("%c", buff[i]);
  }
  printf("finished \n");
}

void Nosr::sdk_test(std::string device_name, int baud) {
  serial_ = SerialStream::CreateSerial(device_name.c_str(), baud);
  if (!serial_ || !serial_->Connect()) {
    std::cout << "cannot upload gga to R9300" << std::endl;
    return;
  }

  int ret = 0;
  /*
   * ** WARNING **
   * PLEASE FIRST CONFIRM THAT YOUR ACCOUNT IS AK OR DSK ?!?
   *
   * If your account is AK (usually with prefix `A`, like: `A00012dwejd`), set
   * the `key_type` to `QXWZ_SDK_KEY_TYPE_AK`. Otherwise, if it is DSK (usually
   * with prefix `D`, like: `D0234jdwejd`), set the `key_type` to
   * `QXWZ_SDK_KEY_TYPE_DSK`.
   * ** WARNING **
   */
  qxwz_sdk_config_t sdk_config;

  /* AK or DSK ? Only choose one in the following codes! */
  /** AK */
  sdk_config.key_type = QXWZ_SDK_KEY_TYPE_AK, strcpy(sdk_config.key, "Your AK");
  strcpy(sdk_config.secret, "Your AS");
  /** DSK */
  sdk_config.key_type = QXWZ_SDK_KEY_TYPE_DSK,
  strcpy(sdk_config.key, "D48k31b9p5p2");
  strcpy(sdk_config.secret, "3dd1f7db5e7ef105");

  // strcpy(sdk_config.key, "D48k34gj56ul");
  // strcpy(sdk_config.secret, "9813c11e34b58ba4");

  /* set device info */
  strcpy(sdk_config.dev_id, "Your device id");
  strcpy(sdk_config.dev_type, "Your device type");

  /* set callbacks */
  sdk_config.status_cb = demo_on_status;
  sdk_config.data_cb = demo_on_data;
  sdk_config.auth_cb = demo_on_auth;
  sdk_config.start_cb = demo_on_start;

  /* If you need special format nosr data(QX format), use the segment*/
  qxwz_sdk_nosr_data_format_t data_fmt = QXWZ_SDK_NOSR_QX_FORMAT;
  ret = qxwz_sdk_config(QXWZ_SDK_CONF_NOSR_DATA_FORMAT, &data_fmt);
  DEMO_LOG("set nosr data format!ret=%d\n", ret);

  unsigned int tick = 0;
  struct timeval tval = {0};
  gettimeofday(&tval, NULL);

  /*
   * init sdk
   */
  ret = qxwz_sdk_init(&sdk_config);
  if (ret < 0) {
    printf("sdk init failed\n");
    goto END;
  }

  /*
   * do authentication
   */
  ret = qxwz_sdk_auth();
  if (ret < 0) {
    printf("call sdk auth failed\n");
    goto END;
  }

  while (1) {
    std::string gga_info;
    gettimeofday(&tval, NULL);
    usleep(100 * 1000); /* 100ms */
    if ((++tick % 10) == 0) {
      std::cout << "sending gga to qx" << std::endl;
      std::cout << g_gga_upload << std::endl;
      qxwz_sdk_upload_gga(g_gga_upload.c_str(), g_gga_upload.size());
    }

    if (sdk_auth_flag > 0) {
      sdk_auth_flag = 0;
      for (int i = 0; i < sdk_cap_info.caps_num; i++) {
        if (sdk_cap_info.caps[i].cap_id == QXWZ_SDK_CAP_ID_NOSR &&
            sdk_cap_info.caps[i].state == QXWZ_SDK_CAP_STATE_INSERVICE) {
          qxwz_sdk_start(QXWZ_SDK_CAP_ID_NOSR); /* start LSSR capability */
        }
      }
    }
  }

  qxwz_sdk_stop(QXWZ_SDK_CAP_ID_NOSR); /* stop NOSR capability */

END:
  qxwz_sdk_cleanup();
}
