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

#include "middle_ware/cyber/message_convert.hpp"
#include <memory>
#include "common/coordinate_transform/LocalCartesian_util.h"
#include "common/coordinate_transform/earth.hpp"
#include "common/coordinate_transform/units.hpp"
// #include "common/coordinate_transform/coordinate_transform_core.h"
#include "common/gnss_third_party/rtklib.h"
#include "modules/common/utils.hpp"
namespace civ {
namespace civloc {
sp_cZImu Convert(crString channel_name, crsp_cKITTI_RAW frame) {
  spZImu imu = std::make_shared<ZImu>();
  imu->t0_ = us2s(frame->timestamp_us);
  imu->t1_ = us2s(frame->timestamp_us);
  imu->channel_name_ = channel_name;
  imu->acc_ = {frame->ax, frame->ay, frame->az};
  imu->gyr_ = {frame->wx, frame->wy, frame->wz};
  imu->channel_name_ = "rawimu";
  return imu;
}

sp_cZImu Convert(crString channel_name, crsp_cIMUProto frame) {
  spZImu imu = std::make_shared<ZImu>();
  frame->linear_acceleration().x();
  imu->t0_ = frame->measurement_time();
  imu->t1_ = frame->measurement_time();
  imu->channel_name_ = channel_name;
  imu->acc_ = {frame->linear_acceleration().y(),
               frame->linear_acceleration().x(),
               -frame->linear_acceleration().z()};
  // imu->acc_ = {frame->linear_acceleration().y(),
  //              frame->linear_acceleration().x(),
  //              -frame->linear_acceleration().z()};
  imu->gyr_ = {0, 0, 0};
  // imu->gyr_ = {frame->angular_velocity().y(), frame->angular_velocity().x(),
  //            -frame->angular_velocity().z()};
  return imu;
}

sp_cZGnss Convert(crString channel_name, crsp_cGNSSProto frame) {
  spZGnss gnss_result = std::make_shared<ZGnss>();
  gnss_result->t0_ = frame->utc_in_seconds();  // 数据产生时刻
  gnss_result->t1_ = frame->utc_in_seconds();  // 数据接受时刻
  gnss_result->channel_name_ = channel_name;
  gnss_result->pos_ = Eigen::Vector3d(
      D2R * frame->latitude_deg(), D2R * frame->longitude_deg(), frame->alt());

  gnss_result->std_pos_ =
      Eigen::Vector3d(frame->hdop(), frame->hdop(), frame->hdop());
  gnss_result->hdop_ = frame->hdop();
  gnss_result->status_ = frame->quality();
  gnss_result->sat_num_ = frame->num_of_satellites_used();
  return gnss_result;
}

LocalizationEstimate Convert(crsp_cState state)  // 定位输出
{
  using namespace civ::common::coord_transform;

  LocalizationEstimate result;
  result.set_measurement_time(state->t0_);
  auto *pose = result.mutable_pose();
  // 1 pos
  pose->mutable_position_llh()->set_lat(state->pos_(0));
  pose->mutable_position_llh()->set_lon(state->pos_(1));
  pose->mutable_position_llh()->set_height(state->pos_(2));

  Eigen::Vector3d pos_enu;
  ConvertLLAToENU(Earth::GetOrigin(),
                  Eigen::Vector3d{state->pos_(0) * R2D, state->pos_(1) * R2D,
                                  state->pos_(2)},
                  &pos_enu);
  // Eigen::Vector3d pos_enu = get_enu_pos(state->pos_);
  pose->mutable_position()->set_x(pos_enu(0));
  pose->mutable_position()->set_y(pos_enu(1));
  pose->mutable_position()->set_z(pos_enu(2));
  // 2 orientation
  pose->mutable_orientation()->set_qw(state->qua_.w());
  pose->mutable_orientation()->set_qx(state->qua_.x());
  pose->mutable_orientation()->set_qy(state->qua_.y());
  pose->mutable_orientation()->set_qz(state->qua_.z());
  return result;
}

sp_cZCnState Convert_cgi610(crString channel_name, crsp_cGNSSProto frame) {
  spZCnState cgi610 = std::make_shared<ZCnState>();
  cgi610->t0_ = frame->utc_in_seconds();
  cgi610->t1_ = frame->utc_in_seconds();
  cgi610->channel_name_ = channel_name;
  // cgi610->pos_ = Eigen::Vector3d(0,
  //                                0, 0);

  cgi610->pos_ = Eigen::Vector3d(D2R * frame->latitude_deg(),
                                 D2R * frame->longitude_deg(), frame->alt());
  cgi610->pos_std_ =
      Eigen::Vector3d(frame->hdop(), frame->hdop(), frame->hdop());
  cgi610->sat_status_ = static_cast<CHC_SATELLITE_STATUS>(frame->quality());
  return cgi610;
}

sp_cZCnState Convert_cgi610(crString channel_name, crsp_cKITTI_RAW frame) {
  spZCnState cgi610 = std::make_shared<ZCnState>();
  cgi610->t0_ = us2s(frame->timestamp_us);
  cgi610->t1_ = us2s(frame->timestamp_us);
  cgi610->channel_name_ = channel_name;
  cgi610->pos_ = {frame->lat, frame->lon, frame->alt};
  cgi610->att_ = Eigen::Vector3d(frame->roll, frame->pitch, frame->yaw);
  cgi610->vel_ = {frame->ve, frame->vn, frame->vu};
  // cgi610->acc_={frame->ax,frame->ay,frame->az};
  // cgi610->gyr_={frame->wx,frame->wy,frame->wz};
  cgi610->sat_status_ = civ::civloc::CHC_SATELLITE_STATUS::RTK_FIXED;

  return cgi610;
}

sp_cZGnss Convert_gnss(crString channel_name, crsp_cKITTI_RAW frame) {
  spZGnss gnss_result = std::make_shared<ZGnss>();
  gnss_result->t0_ = us2s(frame->timestamp_us);  // 数据产生时刻
  gnss_result->t1_ = us2s(frame->timestamp_us);  // 数据接受时刻
  gnss_result->channel_name_ = channel_name;
  // gnss_result->
  Eigen::Vector3d position(frame->lat, frame->lon, frame->alt);  // 位置
  gnss_result->pos_ = position;
  gnss_result->vel_ = Eigen::Vector3d{frame->ve, frame->vn, frame->vu};

  gnss_result->std_vel_.setZero();
  gnss_result->status_ = 5;
  gnss_result->sat_num_ = 10;

  return gnss_result;
}

spState Convert_state(crsp_cKITTI_RAW frame) {
  using namespace civ::common::coord_transform;
  spState state = std::make_shared<State>();
  state->t0_ = us2s(frame->timestamp_us);
  state->t1_ = us2s(frame->timestamp_us);
  state->pos_ = {frame->lat, frame->lon, frame->alt};
  state->qua_ = a2qua(Eigen::Vector3d(frame->roll, frame->pitch, frame->yaw));
  state->vel_ = {frame->ve, frame->vn, frame->vu};
  // state->acc_={frame->ax/ gl_g0,frame->ay/ gl_g0,frame->az/ gl_g0};
  // state->gyr_={frame->wx,frame->wy,frame->wz};

  return state;
}

}  // namespace civloc
}  // namespace civ
