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

#include "common/util/util.h"
#include <experimental/filesystem>
#include <cmath>
#include <vector>

namespace civ {
namespace common {
namespace util {

double const g_ori_pos_deg[3]{31.284156453, 121.170937985, 16.504};

std::vector<std::string> split(const std::string& str,
                               const std::string& pattern) {
  // const char* convert to char*
  char* strc = new char[strlen(str.c_str()) + 1];
  strcpy(strc, str.c_str());
  std::vector<std::string> resultVec;
  char* tmpStr = strtok(strc, pattern.c_str());
  while (tmpStr != NULL) {
    resultVec.push_back(std::string(tmpStr));
    tmpStr = strtok(NULL, pattern.c_str());
  }

  delete[] strc;

  return resultVec;
}

PointENU operator+(const PointENU enu, const math::Vec2d& xy) {
  PointENU point;
  point.set_x(enu.x() + xy.x());
  point.set_y(enu.y() + xy.y());
  point.set_z(enu.z());
  return point;
}

PathPoint GetWeightedAverageOfTwoPathPoints(const PathPoint& p1,
                                            const PathPoint& p2,
                                            const double w1, const double w2) {
  PathPoint p;
  p.set_x(p1.x() * w1 + p2.x() * w2);
  p.set_y(p1.y() * w1 + p2.y() * w2);
  p.set_z(p1.z() * w1 + p2.z() * w2);
  p.set_theta(p1.theta() * w1 + p2.theta() * w2);
  p.set_kappa(p1.kappa() * w1 + p2.kappa() * w2);
  p.set_dkappa(p1.dkappa() * w1 + p2.dkappa() * w2);
  p.set_ddkappa(p1.ddkappa() * w1 + p2.ddkappa() * w2);
  p.set_s(p1.s() * w1 + p2.s() * w2);
  return p;
}


std::vector<std::string> ListSameNameFile(std::string const& path) {
  namespace fs = std::experimental::filesystem;
  std::vector<std::string> all_files;
  fs::path current_path(path);

  AERROR << " current_path.parent_path() =" << current_path.parent_path() << " name with date " << current_path.stem();

  for (auto const& one_path : fs::directory_iterator(current_path.parent_path())) {
    // AINFO << "all sub " << one_path.path();

    if (fs::is_regular_file(one_path.path())) {
      // AINFO << "normal file " << one_path.path().string();
      if (one_path.path().string().find(current_path.stem()) != std::string::npos) {
        // AERROR << one_path.path().string();
        all_files.push_back(one_path.path().string());
      }
    }
  }

  std::sort(all_files.begin(), all_files.end());
  return all_files;
}

std::string get_dir_path(std::string const& path) {
  namespace fs = std::experimental::filesystem;
  fs::path current_path(path);
  return current_path.parent_path();
}

}  // namespace util
}  // namespace common
}  // namespace civ
