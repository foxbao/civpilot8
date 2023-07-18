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

#include <stdint.h>
#include <memory>
#include <string>
namespace civ {
namespace drivers {
namespace gnss {
#define cr(T) T const &
#define sp(T) std::shared_ptr<T>
#define up(T) std::unique_ptr<T>
#define crsp_c(T) std::shared_ptr<T const> const &
#define sp_c(T) std::shared_ptr<T const>
#define crsp(T) std::shared_ptr<T> const &

#define DEFINE_EXTEND_TYPE(T)                         \
  using cr##T = T const &;                            \
  using sp##T = std::shared_ptr<T>;                   \
  using up##T = std::unique_ptr<T>;                   \
  using crsp_c##T = std::shared_ptr<T const> const &; \
  using sp_c##T = std::shared_ptr<T const>;           \
  using crsp##T = std::shared_ptr<T> const &

struct RawImu {
  uint32_t gps_week;
  double gps_seconds;  // Seconds of week.
  uint32_t imuStatus;  // Status of the IMU. The content varies with IMU type.
  int32_t z_velocity_change;      // change in velocity along z axis.
  int32_t y_velocity_change_neg;  // -change in velocity along y axis.
  int32_t x_velocity_change;      // change in velocity along x axis.
  int32_t z_angle_change;         // change in angle around z axis.
  int32_t y_angle_change_neg;     // -change in angle around y axis.
  int32_t x_angle_change;         // change in angle around x axis.
};
DEFINE_EXTEND_TYPE(RawImu);

enum class GNSSmsgType {
  None = 0,
  GNGGA = 1,
  GNVTG = 2,
  GNZDA = 3,
  GNRMC = 4,
};

constexpr uint32_t GNSS_BUFFER_SIZE = 2048;
struct GNSSmsg {
  // actually used buffer length
  uint32_t buffer_len = 0;
  GNSSmsgType type_;
  uint8_t buffer[GNSS_BUFFER_SIZE] = {0};
  uint32_t buffer_size = GNSS_BUFFER_SIZE;
  std::string to_string() const {
    std::string msg;
    msg.assign(reinterpret_cast<const char *>(buffer), buffer_len - 2);
    return msg;
  }
};
DEFINE_EXTEND_TYPE(GNSSmsg);

constexpr uint32_t GNGGA_BUFFER_SIZE = 1024;

struct GNGGA : public GNSSmsg {
  GNGGA() { type_ = GNSSmsgType::GNGGA; }
};
DEFINE_EXTEND_TYPE(GNGGA);

constexpr uint32_t GNRMC_BUFFER_SIZE = 1024;
struct GNRMC : public GNSSmsg {
  GNRMC() { type_ = GNSSmsgType::GNRMC; }
};
DEFINE_EXTEND_TYPE(GNRMC);

constexpr uint32_t RTCM_BUFFER_SIZE = 2048;

struct RTCM {
  uint8_t buffer[RTCM_BUFFER_SIZE] = {0};
  uint32_t buffer_len = 0;
  uint32_t buffer_size = RTCM_BUFFER_SIZE;
};
DEFINE_EXTEND_TYPE(RTCM);

}  // namespace gnss
}  // namespace drivers
}  // namespace civ
