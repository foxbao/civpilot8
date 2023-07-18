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

#include "Converter.hpp"
#include <math.h>
#include <iostream>
#include <string>

namespace civ {
namespace civloc {
std::string binaryToHex(uint8_t data) {
  std::string ret;
  static const char *hex = "0123456789ABCDEF";
  ret.push_back(hex[(data >> 4) & 0xf]);  // 取二进制高四位
  ret.push_back(hex[data & 0xf]);         // 取二进制低四位
  return ret;
}

bool Convert(u8 *Fd_data, char *data, int length) {
  u8 Count = 0;
  for (int i = 0; i < length; i++) {
    Fd_data[i] = (uint8_t)data[i];
    // if((uint8_t)data[i]==FRAME_HEAD)
    // {
    //     if((uint8_t)data[i+1])
    //     int aaa=1;
    // }
  }
  // if ((uint8_t)data[0] == FRAME_HEAD)
  // {
  //     Fd_data[Count] = (uint8_t)data[0];
  //     std::cout << binaryToHex(Fd_data[Count]) << std::endl;
  //     Count++;
  //     Fd_data[Count] = (uint8_t)data[Count];
  //     std::cout << binaryToHex(Fd_data[Count]) << std::endl;
  //     if (Fd_data[1] == TYPE_IMU)
  //     {
  //         int bbbb = 2;
  //     }
  //     if (Fd_data[1] == TYPE_AHRS)
  //     {
  //         int ccc = 3;
  //     }
  //     int aaa = 1;
}

long long timestamp(u8 Data_1, u8 Data_2, u8 Data_3, u8 Data_4) {
  u32 transition_32;
  transition_32 = 0;
  transition_32 |= Data_4 << 24;
  transition_32 |= Data_3 << 16;
  transition_32 |= Data_2 << 8;
  transition_32 |= Data_1;
  return transition_32;
}

float DATA_Trans(u8 Data_1, u8 Data_2, u8 Data_3, u8 Data_4) {
  long long transition_32;
  float tmp = 0;
  int sign = 0;
  int exponent = 0;
  float mantissa = 0;
  transition_32 = 0;
  transition_32 |= Data_4 << 24;
  transition_32 |= Data_3 << 16;
  transition_32 |= Data_2 << 8;
  transition_32 |= Data_1;
  sign = (transition_32 & 0x80000000) ? -1 : 1;  // sign
  exponent = ((transition_32 >> 23) & 0xff) - 127;
  mantissa = 1 + (static_cast<float>(transition_32 & 0x7fffff) / 0x7fffff);
  tmp = sign * mantissa * pow(2, exponent);
  return tmp;
}
}  // namespace civloc
}  // namespace civ
