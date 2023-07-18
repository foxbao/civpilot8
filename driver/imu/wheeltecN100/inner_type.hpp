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
namespace civ {
namespace civloc {

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

#define FRAME_HEAD 0xfc
#define FRAME_END 0xfd
#define TYPE_IMU 0x40
#define TYPE_AHRS 0x41
#define TYPE_INSGPS 0x42
#define TYPE_GROUND 0xf0
#define IMU_LEN 0x38   // 56+8  8
#define AHRS_LEN 0x30  // 48+8  7

typedef uint32_t u32;
typedef uint16_t u16;
typedef uint8_t u8;

#define IMU_RS 64
#define AHRS_RS 56
#define INSGPS_RS 80

typedef struct IMUData_Packet_t {
  float gyroscope_x;           // unit: rad/s
  float gyroscope_y;           // unit: rad/s
  float gyroscope_z;           // unit: rad/s
  float accelerometer_x;       // m/s^2
  float accelerometer_y;       // m/s^2
  float accelerometer_z;       // m/s^2
  float magnetometer_x;        // mG
  float magnetometer_y;        // mG
  float magnetometer_z;        // mG
  float imu_temperature;       // C
  float Pressure;              // Pa
  float pressure_temperature;  // C
  u32 Timestamp;               // us
} IMUData_Packet_t;
DEFINE_EXTEND_TYPE(IMUData_Packet_t);

typedef struct AHRSData_Packet_t {
  float RollSpeed;     // unit: rad/s
  float PitchSpeed;    // unit: rad/s
  float HeadingSpeed;  // unit: rad/s
  float Roll;          // unit: rad
  float Pitch;         // unit: rad
  float Heading;       // unit: rad
  float Qw;            // w          //Quaternion
  float Qx;            // x
  float Qy;            // y
  float Qz;            // z
  u32 Timestamp;       // unit: us
} AHRSData_Packet_t;
DEFINE_EXTEND_TYPE(IMUData_Packet_t);

}  // namespace civloc
}  // namespace civ
