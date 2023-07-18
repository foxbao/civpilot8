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
#include <eigen3/Eigen/Core>
#include <cmath>
#include <string>
#include <vector>
#include <memory>
#include "modules/common/inner_types.hpp"

namespace civ {
namespace civloc {
extern Eigen::Matrix3d gl_I33;
extern Eigen::Matrix3d gl_O33;
// 状态
class StateServer;
class Cacher;
// 全局对象
extern std::shared_ptr<StateServer> g_state_server;  // 状态全局访问
extern std::shared_ptr<LocatorConfig> g_config;      // 配置文件全局访问
extern std::shared_ptr<Cacher> g_cacher;             // 消息缓存
extern std::shared_ptr<SensorsSetting> g_sensor_config;  // 传感器配置

std::ostream &operator<<(std::ostream &os, ZFrameType const &type);
// std::ostream &operator<<(std::ostream &os, MeasurementType const &type);
// std::ostream &operator<<(std::ostream &os, LineSegmentType const &type);
// std::ostream &operator<<(std::ostream &os, LineSegmentColor const &type);
// std::ostream &operator<<(std::ostream &os, PoleType const &type);
// std::ostream &operator<<(std::ostream &os, SignType const &type);

// Extrinsic -> Isometry3d
Eigen::Isometry3d Expara2Isometry3d(Extrinsic const &raw);

std::ostream &operator<<(std::ostream &os, crsp_cZFrame frame);

// 给出Vector3d 1 2 元素为X Y 的坐标差 3为yaw误差
// std::string ComparePosAndAngle(crsp_cZCnState cgi610, crsp_cZLidarMatch
// frame);
Eigen::Vector3d ComparePosAndAngle(crsp_cZCnState cgi610, crsp_cState state);

#ifdef __BUILD_FILESYSTEM_
std::vector<std::string> ListSameNameFile(std::string const& path);
std::string get_dir_path(std::string const& path);
#endif

// zframe 排序
bool operator<(crsp_cZFrame left, crsp_cZFrame right);
bool operator>(crsp_cZFrame left, crsp_cZFrame right);

// zframe 转string
std::string to_string(crsp_cZFrame frame);
std::string to_string_header(crsp_cZFrame frame);
std::string to_rtkplot_string(crsp_cZCnState frame);
std::string to_rtkplot_string(crsp_cState frame);


extern std::vector<std::string> split(const std::string &str,
                                      const std::string &pattern);
// extern std::string trimstr(std::string s);

// extern OCLong hms2unix(const std::string &data);

inline size_t s2us(const double t_s) {
  return static_cast<size_t>(round(t_s * 1.0e6));
}
inline size_t s2ms(const double t_s) {
  return static_cast<size_t>(round(t_s * 1.0e3));
}
inline double ms2s(double t_ms) { return static_cast<double>(t_ms * 1.0e-3); }
inline double us2s(double t_us) { return static_cast<double>(t_us * 1.0e-6); }

// 差值 coff is [0,1]
template <typename T>
T Interpolate(double coff, T const &y1, T const &y2) {
  return y1 + coff * (y2 - y1);
}
sp_cZCnState Interpolate(crsp_cZCnState f1, crsp_cZCnState f2, double t_s);

// return if nullptr
#if !defined(RETURN_IF_NULLPTR)
#define RETURN_IF_NULLPTR(ptr) \
  if (ptr == nullptr) {        \
    return;                    \
  }
#endif

// 序号访问简化
#define kIdxQnn1 (State::idx_.qnn1_)
#define kIdxVel (State::idx_.v_)
#define kIdxPos (State::idx_.p_)
#define kIdxBg (State::idx_.bg_)
#define kIdxBa (State::idx_.ba_)
#define kIdxQvv1 (State::idx_.qvv1_)
#define kIdxKod (State::idx_.kod_)
#define kNS (State::idx_.NS_)
}  // namespace civloc
}  // namespace civ
