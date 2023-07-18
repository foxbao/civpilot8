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
#include "modules/common/utils.hpp"
#include <eigen3/Eigen/Dense>
#ifdef __BUILD_FILESYSTEM_
#include <experimental/filesystem>
#endif
// #include "modules/common/earth.hpp"
#include "common/coordinate_transform/earth.hpp"
#include "modules/common/time_system.hpp"
// #include "modules/common/units.hpp"
namespace civ {
namespace civloc {

Eigen::Matrix3d gl_I33 = Eigen::Matrix3d::Identity();
Eigen::Matrix3d gl_O33 = Eigen::Matrix3d::Zero();

std::ostream &operator<<(std::ostream &os, ZFrameType const &type) {
  switch (type) {
    case ZFrameType::None:
      os << "FT::None";
      break;
    case ZFrameType::IMU:
      os << "FT::IMU";
      break;
    case ZFrameType::WHEEL_ODO:
      os << "FT::WHEEL_ODO";
      break;
    case ZFrameType::GNSS_POS:
      os << "FT::GNSS_POS";
      break;
    case ZFrameType::GNSS_VEL:
      os << "FT::GNSS_VEL";
      break;
    case ZFrameType::GNSS_ALL:
      os << "FT::GNSS_ALL";
      break;
    case ZFrameType::VEHICLE_MODEL:
      os << "FT::VEHICLE_MODEL";
      break;
    case ZFrameType::CGI610:
      os << "FT::CGI610 ";
      break;
    case ZFrameType::STATE:
      os << "FT::STATE ";
      break;
    case ZFrameType::PERCEPTION:
      os << "FT::PERCEPTION ";
      break;
    case ZFrameType::SEMANTIC_MAP:
      os << "FT::SEMANTIC_MAP ";
      break;
    case ZFrameType::LINESEGMENT:
      os << "FT::LINESEGMENT ";
      break;
    case ZFrameType::SIGN:
      os << "FT::SIGN ";
      break;
    case ZFrameType::POLE:
      os << "FT::POLE ";
      break;
    case ZFrameType::SEMANTIC_MATCH:
      os << "FT::SEMANTIC_MATCH ";
      break;
    case ZFrameType::LIDAR_MATCH:
      os << "FT::LIDAR_MATCH";
      break;
    default:
      os << " default ";
      break;
  }
  return os;
}

// Extrinsic -> Isometry3d
Eigen::Isometry3d Expara2Isometry3d(Extrinsic const &raw) {
  Eigen::Isometry3d trans;
  Eigen::Vector3d pos = {raw.translation().x(), raw.translation().y(),
                         raw.translation().z()};
  Eigen::Quaterniond qua = Eigen::Quaterniond::Identity();
  qua.x() = raw.rotation().qx();
  qua.y() = raw.rotation().qy();
  qua.z() = raw.rotation().qz();
  qua.w() = raw.rotation().qw();
  trans.translation() = pos;
  trans.linear() = qua.toRotationMatrix();
  return trans;
}

#ifdef __BUILD_FILESYSTEM_
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
#endif

bool operator<(crsp_cZFrame left, crsp_cZFrame right) {
  // AERROR << "<<< bool operator<(crsp_cZFrame left, crsp_cZFrame right)
  // called!!!!!!!!!!!!!!!!!"; 相等
  if (fabs(left->t0_ - right->t0_) < 1e-7) {
    return left->type_ < right->type_;
  } else {
    return left->t0_ - right->t0_ < -1e-7;
  }
}

bool operator>(crsp_cZFrame left, crsp_cZFrame right) {
  // AERROR << ">>> bool operator>(crsp_cZFrame left, crsp_cZFrame right)
  // called!!!!!!!!!!!!!!!!!"; 相等
  if (fabs(left->t0_ - right->t0_) < 1e-7) {
    return left->type_ > right->type_;
  } else {
    return left->t0_ - right->t0_ > 1e-7;
  }
}

std::vector<std::string> split(const std::string &str,
                               const std::string &pattern) {
  // const char* convert to char*
  char *strc = new char[strlen(str.c_str()) + 1];
  strcpy(strc, str.c_str());
  std::vector<std::string> resultVec;
  char *tmpStr = strtok(strc, pattern.c_str());
  while (tmpStr != NULL) {
    resultVec.push_back(std::string(tmpStr));
    tmpStr = strtok(NULL, pattern.c_str());
  }

  delete[] strc;

  return resultVec;
}

sp_cZCnState Interpolate(crsp_cZCnState f1, crsp_cZCnState f2, double t_s) {
  using namespace civ::common::coord_transform;
  auto result = std::make_shared<ZCnState>(*f1);
  double coff = (t_s - f1->t0_) / (f2->t0_ - f1->t0_);

  result->t0_ = t_s;
  result->pos_ = Interpolate(coff, f1->pos_, f2->pos_);
  result->vel_ = Interpolate(coff, f1->vel_, f2->vel_);
  Eigen::Quaterniond qt =a2qua(f1->att_).slerp(coff, a2qua(f2->att_));
  result->att_ = q2att(qt);
  result->acc_in_vehicle_ =
      Interpolate(coff, f1->acc_in_vehicle_, f2->acc_in_vehicle_);
  result->paltance_in_vehicle_ =
      Interpolate(coff, f1->paltance_in_vehicle_, f2->paltance_in_vehicle_);
  result->gyr_ = Interpolate(coff, f1->gyr_, f2->gyr_);
  result->acc_ = Interpolate(coff, f1->acc_, f2->acc_);
  return result;
}

sp_cZImu Interpolate(crsp_cZImu f1, crsp_cZImu f2, double t_s) {
  auto result = std::make_shared<ZImu>(*f1);
  double coff = (t_s - f1->t0_) / (f2->t0_ - f1->t0_);

  result->t0_ = t_s;
  result->gyr_ = Interpolate(coff, f1->gyr_, f2->gyr_);
  result->acc_ = Interpolate(coff, f1->acc_, f2->acc_);
  return result;
}

}  // namespace civloc
}  // namespace civ
