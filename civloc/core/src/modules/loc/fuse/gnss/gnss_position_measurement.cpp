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

#include "modules/loc/fuse/gnss/gnss_position_measurement.hpp"
namespace civ {
namespace civloc {
bool GnssPositionMeasurement::ConfirmValidity() {
  auto const &gnss_cfg = g_config->fuse_cfg().gnss_filter();
  if (!gnss_cfg.enable_gnss_pos()) {
    return false;
  }
  // pos的std==0的时候拒绝使用改组数据
  if ((frame_->std_pos_(0) > gnss_cfg.pos_std_m_upper_limit()) ||
      (frame_->std_pos_(1) > gnss_cfg.pos_std_m_upper_limit()) ||
      (frame_->std_pos_(2) > gnss_cfg.pos_std_m_upper_limit())) {
    AERROR << "gnss pos std large than  100m"
           << " " << UnixTimeString(frame_->t0_) << " e:" << frame_->std_pos_(0)
           << " n:" << frame_->std_pos_(1) << " u:" << frame_->std_pos_(2);
    return false;
  }

  return true;
}

void GnssPositionMeasurement::AdaptNoise() {
  // 自适应调整R
  using namespace Eigen;
  auto const& gnss_cfg = g_config->fuse_cfg().gnss_filter();
  //
  static int index[6] = {0, 0, 1, 0, 3, 2};  // 解状态对应的std_pos值 修改映射关系，避免异常情况发生
  double std = gnss_cfg.position_noise_m()[index[frame_->status_]];
  Vector3d pos_std_chi2 = {std, std, std};
  R_ = DiagonalMatrix<double, 3>(pos_std_chi2).toDenseMatrix().array().pow(2);
  // AdaptPseudoRangeOrSiglePointNoise();
  // AdaptFloatNoise();
  // AdaptRtkNoise();
}

void GnssPositionMeasurement::ComputeJacobian() {
  using namespace civ::common::coord_transform;
  Eigen::Matrix3d const cnv = pre_state_->c_nv();
  Eigen::Vector3d const t_v_ai =
      pre_state_->T_vi().translation() -
      pre_state_->T_vg().translation();  // 天线到IMU向量，在车体系下的分解
  Eigen::Matrix3d const lv_x = askew(t_v_ai);        // v系杆臂反对称
  Eigen::Matrix3d const ln_x = askew(cnv * t_v_ai);  // v系杆臂反对称
  Eigen::Vector3d const p_wa = Earth::PlusDeltaEnuAtPos(
      pre_state_->pos_, -cnv * t_v_ai);  // IMU观测值折算至天线相位中心

  // H
  H_ = Eigen::MatrixXd::Zero(3, kNS);
  H_.leftCols(15) << -ln_x, gl_O33, gl_I33, Eigen::MatrixXd::Zero(3, 6);
  if (g_config->fuse_cfg().online_estimation_qvv1()) {
    H_.block<3, 3>(0, kIdxQvv1) = cnv * lv_x;
  }

  // Jcobian简化
  // if (frame_->status_ != 4) {
  //   H_.setZero();
  //   H_.block<3, 3>(0, kIdxPos) = gl_I33;
  // }

  // Z
  Z_ = Earth::DeltaPosEnuInFirstPoint(p_wa, frame_->pos_);
}

bool GnssPositionMeasurement::ChiSquareTest() {
  return true;
}
}  // namespace civloc
}  // namespace civ
