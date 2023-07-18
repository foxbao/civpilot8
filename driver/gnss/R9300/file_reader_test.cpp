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

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include "gnss/R9300/parser/R9300_ascii_parser.hpp"
// #include "gnss/R9300/stream/stream.h"
void output_buff(char *buff, int size) {
  printf("remain size: %d\n", size);
  for (int i = 0; i < size; i++) {
    printf("%c", buff[i]);
  }
  printf("\n");
}

int main(int argc, char *argv[]) {
  std::string gps_file_name =
      "/home/baojiali/Downloads/civpilot/driver/gnss/R9300/gpslog.txt";
  std::ifstream ifs(gps_file_name, std::ifstream::in);

  using civ::drivers::R9300::R9300AsciiParser;
  using civ::drivers::gnss::GNSSmsg;
  R9300AsciiParser parser;
  // R9300Stream r9300_stream;

  int buff_size = 2048;
  char* buff = new char[buff_size];  // the buffer that is to be parsed
  int remainSize = 0;

  int reading_size = 0;  // data size that are read from file
  // char data[reading_size];

  if (!ifs.is_open()) {
    return 1;
  }
  while (!ifs.eof()) {
    output_buff(buff, remainSize);
    int reading_size = buff_size - remainSize;
    // read data from file and store them in buff, from remain position
    size_t length = ifs.read(buff + remainSize, reading_size).gcount();

    if (length) {
      // current length of data contained in buff: remain+new
      length += remainSize;
      parser.UpdateDataPtr(buff, length);
      std::vector<GNSSmsg> v_gnss;
      parser.ParseData(v_gnss);
      remainSize = parser.RemainSize();

      // if the size of data not parsed is less than the buff size
      if (remainSize < length && remainSize > 0) {
        // move the remaining part from tail to head
        memcpy(buff, buff + length - remainSize, remainSize);
      }
    } else {
      std::cout << "read no data from file" << std::endl;
    }

    int a = 1;
  }

  ifs.close();
  // std::shared_ptr<FILE> file=std::make_shared<FILE>();

  // file = fopen("gpslog.txt", "rb");
  // int aaa=1;
}
