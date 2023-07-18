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

#include "middle_ware/cyber/tool/preprocessor.hpp"
#include <dirent.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <algorithm>
#include <ctime>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "modules/common/time_system.hpp"
#include "modules/common/utils.hpp"

namespace civ {
namespace civloc {
bool get_all_files(const std::string &dir_in, std::vector<std::string> &files) {
  if (dir_in.empty()) {
    return false;
  }
  struct stat s;
  stat(dir_in.c_str(), &s);
  if (!S_ISDIR(s.st_mode)) {
    return false;
  }
  DIR *open_dir = opendir(dir_in.c_str());
  if (NULL == open_dir) {
    std::exit(EXIT_FAILURE);
  }
  dirent *p = nullptr;
  while ((p = readdir(open_dir)) != nullptr) {
    struct stat st;
    if (p->d_name[0] != '.') {
      // 因为是使用devC++ 获取windows下的文件，所以使用了 "\" ,linux下要换成"/"
      std::string name = dir_in + std::string("/") + std::string(p->d_name);
      stat(name.c_str(), &st);
      if (S_ISDIR(st.st_mode)) {
        get_all_files(name, files);
      } else if (S_ISREG(st.st_mode)) {
        files.push_back(name);
      }
    }
  }
  closedir(open_dir);
  return true;
}
DataReader::DataReader() { int aaaa = 2; }

DataReader::~DataReader() {}

void DataReader::ReadKittiTimeStamps(const std::string &timestamps_file,
                                     std::vector<int64_t> &timestamps_us) {
  std::ifstream inFile;
  inFile.open(timestamps_file);
  if (!inFile) {
    std::cerr << "Unable to open file datafile.txt";
    exit(1);  // call system to stop
  }
  std::string item;
  int count = 0;
  while (inFile) {
    if (!getline(inFile, item)) {
      break;
    }
    int64_t timestamp_us = Str2Unix(item);
    // AINFO << count << ": " << item << " unix time us:" << timestamp_us <<
    // endl;
    timestamps_us.push_back(timestamp_us);
    ++count;
  }
  inFile.close();
}

void DataReader::ReadKittiRawData(const std::string &raw_file,
                                  std::vector<KITTI_RAW> &v_kitti_data) {
  std::vector<std::string> files;
  get_all_files(raw_file, files);

  sort(files.begin(), files.end());

  int count = 0;
  for (const auto &rawFile : files) {
    std::ifstream inFile;
    inFile.open(rawFile);
    if (!inFile) {
      std::cerr << "Unable to open file datafile.txt";
      exit(1);  // call system to stop
    }
    std::string item;
    int count = 0;
    while (inFile) {
      if (!getline(inFile, item)) {
        break;
      }
      KITTI_RAW data = ConvertKittiRaw(item);
      v_kitti_data.push_back(data);
      ++count;
    }
    inFile.close();
  }
}

KITTI_RAW DataReader::ConvertKittiRaw(std::string &str_data) {
  KITTI_RAW data;
  std::vector<std::string> split_result = split(str_data, " ");
  int aaa = 1;
  data.lat = stof(split_result[0]);
  data.lon = stof(split_result[1]);
  data.alt = stof(split_result[2]);
  data.roll = stof(split_result[3]);
  data.pitch = stof(split_result[4]);
  data.yaw = stof(split_result[5]);
  data.vn = stof(split_result[6]);
  data.ve = stof(split_result[7]);
  data.vf = stof(split_result[8]);
  data.vl = stof(split_result[9]);
  data.vu = stof(split_result[10]);
  data.ax = stof(split_result[11]);
  data.ay = stof(split_result[12]);
  data.az = stof(split_result[13]);
  data.af = stof(split_result[14]);
  data.al = stof(split_result[15]);
  data.au = stof(split_result[16]);
  data.wx = stof(split_result[17]);
  data.wy = stof(split_result[18]);
  data.wz = stof(split_result[19]);
  data.wf = stof(split_result[20]);
  data.wl = stof(split_result[21]);
  data.wu = stof(split_result[22]);
  data.pos_accuracy = stof(split_result[23]);
  data.vel_accuracy = stof(split_result[24]);
  data.navstat = stof(split_result[25]);
  data.numsats = stof(split_result[26]);
  data.posmode = stof(split_result[27]);
  data.velmode = stof(split_result[28]);
  data.orimode = stof(split_result[29]);

  return data;
}
}  // namespace civloc
}  // namespace civ
