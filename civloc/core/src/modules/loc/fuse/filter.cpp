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

#include "modules/loc/fuse/filter.hpp"
#include <fstream>
#include "common/coordinate_transform/coordinate_transform_core.h"
#include "common/interface/logger/log_interface.h"
#include "modules/common/utils.hpp"
#include "modules/message_cache/cacher.hpp"
#include "modules/state_server/state_server.hpp"

namespace civ {
namespace civloc {

spEKF g_eskf = nullptr;

StateIndex State::idx_ = StateIndex();  // 初始化状态索引
EKF::EKF() {
  if (DetermineSensors()) {
    DetermineIndex();
    ConfirmProgressNoise();
  }
}

Eigen::MatrixXd &EKF::ResetH(size_t const row, size_t const col) {
  H_ = Eigen::MatrixXd::Zero(row, col);
  return H_;
}

double EKF::ComputeChiSquare(crsp_cState state) {
  Eigen::MatrixXd const &Pk = state->cov_;
  innov_var_inverse_ = (H_ * Pk * H_.transpose() + R_).inverse();
  double chi_square_test_value = Z_.transpose() * innov_var_inverse_ * Z_;
  return chi_square_test_value;
}

Eigen::VectorXd EKF::ComputeDeltaX(crsp_cState state) {
  Eigen::MatrixXd const &Pk = state->cov_;
  // 计算卡尔曼增益
  K_ = Pk * H_.transpose() * innov_var_inverse_;
  // 计算修正量
  dx_ = K_ * Z_;
  return dx_;
}

void EKF::UpdateP(spState state) {
  Eigen::MatrixXd &Pk = state->cov_;
  Pk = Pk - K_ * H_ * Pk;            // 更新协方差阵
  Pk = 0.5 * (Pk + Pk.transpose());  // 强制对称
}

void EKF::CompensateDeltaX(spState state, Eigen::VectorXd const &dx) {
  // 姿态反馈
  using namespace civ::common::coord_transform;
  state->qua_ = rv2q(dx.segment<3>(kIdxQnn1)) * state->qua_;
  state->qua_.normalize();
  // 速度反馈
  state->vel_ = state->vel_ - dx.segment<3>(kIdxVel);
  // 位置反馈
  state->pos_ = Earth::PlusDeltaEnuAtPos(state->pos_, -dx.segment<3>(kIdxPos));
  // IMU 内参反馈
  state->imu_in_para_.bg_ += dx.segment<3>(kIdxBg);
  // 加计零偏反馈
  state->imu_in_para_.ba_ += dx.segment<3>(kIdxBa);
  // imu安装角
  if (g_config->fuse_cfg().online_estimation_qvv1()) {
    Eigen::Quaterniond qvi =
        rv2q(dx.segment<3>(kIdxQvv1)) *
        Eigen::Quaterniond(state->ex_para_["imu"].linear());
    qvi.normalize();
    state->ex_para_["imu"].linear() = qvi.toRotationMatrix();
  }
  // 轮速刻度因子
  if (g_config->fuse_cfg().online_estimation_ko()) {
    state->odo_in_para_ =
        state->odo_in_para_.array() * (1 - dx.segment<4>(kIdxKod).array());
  }

  Eigen::RowVectorXd dx2 = dx.transpose();
  if (g_config->fuse_cfg().online_estimation_qvv1()) {
    AINFO << std::fixed << std::setprecision(3) << "delta "
          << dx2.segment<3>(kIdxPos) << " m " << dx2.segment<3>(kIdxVel)
          << " m/s @@@ " << dx2.segment<3>(kIdxQnn1) / gl_deg << " ° !!! "
          << dx2.segment<3>(kIdxBg) / gl_dps << " °/s "
          << dx2.segment<3>(kIdxBa) / gl_mg << " "
          << "  mg !!! " << dx2.segment<3>(kIdxQvv1) / gl_deg << " °";
  } else {
    AINFO << std::fixed << std::setprecision(3) << "delta "
          << dx2.segment<3>(kIdxPos) << " m " << dx2.segment<3>(kIdxVel)
          << " m/s @@@ " << dx2.segment<3>(kIdxQnn1) / gl_deg << " ° !!! "
          << dx2.segment<3>(kIdxBg) / gl_dps << " °/s "
          << dx2.segment<3>(kIdxBa) / gl_mg << " "
          << "  mg !!! ";
  }
  if (g_config->fuse_cfg().online_estimation_ko()) {
    // AERROR << std::fixed << std::setprecision(3) << "Wheel Ko : " <<
    // state->odo_in_para_.transpose();
  }
}

bool EKF::DetermineSensors() { return true; }

void EKF::DetermineIndex() {
  // 经典15维度状态
  State::idx_.qnn1_ = 0;
  State::idx_.v_ = 3;
  State::idx_.p_ = 6;
  State::idx_.bg_ = 9;
  State::idx_.ba_ = 12;
  // 基础状态维度
  State::idx_.NS_ = 15;
  // qvv1
  if (g_config->fuse_cfg().online_estimation_qvv1()) {
    State::idx_.qvv1_ = kNS;  // 3
    State::idx_.NS_ += 3;
  }
  if (g_config->fuse_cfg().online_estimation_ko()) {
    State::idx_.kod_ = kNS;
    State::idx_.NS_ += 4;
  }

  // 初始化缓存区
  trans_noise_ = Eigen::MatrixXd::Zero(kNS, kNS);
  trans_phi_ = Eigen::MatrixXd::Identity(kNS, kNS);

  // 初始化全局对象
  std::cout << " NS = " << State::idx_.NS_ << std::endl;
  int aaa = 1;
}

void EKF::ConfirmProgressNoise() {
  using namespace civ::common::coord_transform;
  auto const &imu_para = g_sensor_config->imu(0);
  auto const &acc_para = imu_para.acc();
  auto const &gyr_para = imu_para.gyr();
  auto const &filter_para = g_config->fuse_cfg().filter_para();
  //
  qt_.setZero(kNS, 1);
  //
  qt_.segment<3>(kIdxQnn1) =
      Eigen::Map<Eigen::Vector3d const>(gyr_para.angle_random_walk().data()) *
      gl_dpsh;
  qt_.segment<3>(kIdxVel) = Eigen::Map<Eigen::Vector3d const>(
                                acc_para.velocity_random_walk().data()) *
                            gl_mpsh;
  qt_.segment<3>(kIdxPos) =
      Eigen::Map<Eigen::Vector3d const>(
          filter_para.position_random_walk_mpsh().data()) *
      gl_mpsh;

  // 相关时间
  double const acc_related_time_s =
      g_sensor_config->imu(0).acc().related_time_s();
  double const gyr_related_time_s =
      g_sensor_config->imu(0).gyr().related_time_s();
  if (gyr_related_time_s >= 0) {
    const double coff = gl_dph * sqrt(2 / gyr_related_time_s);
    qt_.segment<3>(kIdxBg) =
        Eigen::Map<Eigen::Vector3d const>(gyr_para.bias_stability().data()) *
        coff;
  }
  if (acc_related_time_s >= 0) {
    const double coff = gl_ug * sqrt(2 / acc_related_time_s);
    qt_.segment<3>(kIdxBa) =
        Eigen::Map<Eigen::Vector3d const>(acc_para.bias_stability().data()) *
        coff;
  }
  if (g_config->fuse_cfg().online_estimation_ko()) {
    qt_.segment<4>(kIdxKod) = Eigen::Map<Eigen::Vector4d const>(
        filter_para.odo_scale_random_walk_psh().data());
  }
  qt_ = qt_.cwiseAbs2();
  // 改
  AINFO << "qt = " << qt_.transpose();
  // std::cout<<"qt = " << qt_.transpose()<<std::endl;
}

void EKF::PredictCovarianceByIMU(crsp_cState filter_state, crsp_cZFrame frame,
                                 crsp_cZImu imu, spState predicted_state) {
  using namespace civ::common::coord_transform;
  double dt = frame->t0_ - filter_state->t0_;

  Eigen::MatrixXd F = Eigen::MatrixXd::Zero(kNS, kNS);
  Eigen::MatrixXd const &Pii = filter_state->cov_;
  // Vector3d const fn = filter_state->qua_ * imu->acc_ * gl_g0; //
  Eigen::Vector3d const fn = filter_state->qua_ * imu->acc_;  //

  Eigen::Matrix3d cni = filter_state->qua_.toRotationMatrix();
  F.block<3, 3>(kIdxQnn1, kIdxBg) = -cni;
  F.block<3, 3>(kIdxVel, kIdxQnn1) = askew(fn);
  F.block<3, 3>(kIdxVel, kIdxBa) = cni;
  F.block<3, 3>(kIdxPos, kIdxVel) = gl_I33;
  // 相关时间
  double const acc_related_time_s =
      g_sensor_config->imu(0).acc().related_time_s();
  double const gyr_related_time_s =
      g_sensor_config->imu(0).gyr().related_time_s();
  if (gyr_related_time_s > 0) {
    F.block<3, 3>(kIdxBg, kIdxBg) = gl_I33 * (-1 / gyr_related_time_s);
  }
  if (acc_related_time_s > 0) {
    F.block<3, 3>(kIdxBa, kIdxBa) = gl_I33 * (-1 / acc_related_time_s);
  }

  // 状态转移矩阵
  Eigen::MatrixXd Phi = Eigen::MatrixXd::Identity(kNS, kNS) + F * dt;
  // 状态转移噪声, 对角阵,一下转换无用 qt需要初始化
  Eigen::MatrixXd Q = qt_.asDiagonal() * dt;
  // 协方差延迟转移
  // if (frame->type_ == ZFrameType::IMU) {
  //   //进行一次累加计算
  //   trans_phi_ = Phi * trans_phi_;
  //   trans_noise_ = Phi * trans_noise_ * Phi.transpose();
  //   trans_noise_ = 0.5 * (trans_noise_ + trans_noise_.transpose());  //对角化
  //   trans_noise_ += Q;
  // } else {
  //   predicted_state->cov_ = trans_phi_ * Pii * trans_phi_.transpose() +
  //   trans_noise_; predicted_state->cov_ = 0.5 * (predicted_state->cov_ +
  //   predicted_state->cov_.transpose());
  //   // 重置缓存
  //   trans_phi_.setIdentity();
  //   trans_noise_.setZero();
  // }
  // 协方差延迟转移
  predicted_state->cov_ = Phi * Pii * Phi.transpose() + Q;
  // P阵对角化
  predicted_state->cov_ =
      0.5 * (predicted_state->cov_ + predicted_state->cov_.transpose());
}
using namespace Eigen;
using namespace civ::common::coord_transform;
void EKF::PredictStateByIMU(crsp_cState filter_state, crsp_cZFrame frame,
                            crsp_cZImu imu, spState predicted_state) {
  // std::cout << "filter_state->pos_" << filter_state->pos_.transpose()
  //           << std::endl;
  double dt = frame->t0_ - filter_state->t0_;
  Vector3d wnie = Earth::GetWnie(filter_state->pos_),
           wnen = Earth::GetWnen(filter_state->pos_, filter_state->vel_);

  Vector3d const vel_inc = imu->acc_ * dt;
  Vector3d const ang_inc =
      imu->gyr_ * dt;  // 可以使用差值结果  //z轴结果进行了调整

  // 速度更新
  Vector3d wnin = wnie + wnen;
  Quaterniond q_nn1 = rv2q(-wnin * dt);
  Vector3d dvn = q_nn1 * filter_state->qua_ * vel_inc;

  Vector3d dv_gcor = (-(wnie + wnin).cross(filter_state->vel_) +
                      Earth::GetGn(filter_state->pos_)) *
                     dt;  //重力及哥氏加速度
  // auto dv_2 = (dvn + dv_gcor);
  // std::cout<<"dv_2:"<<dv_2.transpose()<<std::endl;
  Vector3d vel_last = filter_state->vel_;
  predicted_state->vel_ = filter_state->vel_ + (dvn + dv_gcor);
  // std::cout << "predicted_state->vel_:" << predicted_state->vel_.transpose()
  //           << std::endl;
  // 位置更新
  Matrix3d cen = Earth::Pos2Cne(filter_state->pos_).transpose();
  predicted_state->pos_ =
      Earth::ECEF2LLH(Earth::LLH2ECEF(filter_state->pos_) +
                      cen * (vel_last + predicted_state->vel_) / 2 * dt);

  Vector3d predicted_enu_pos = get_enu_pos(predicted_state->pos_);
  // std::cout << "predicted_state->pos_:" << predicted_state->pos_.transpose()
  //           << std::endl;
  std::cout << "predicted_enu_pos:" << predicted_enu_pos.transpose()
            << std::endl;

  // 姿态更新
  predicted_state->qua_ = q_nn1 * filter_state->qua_ * rv2q(ang_inc);
  predicted_state->qua_.normalize();

  // 时间更新
  predicted_state->t0_ = frame->t0_;
}

void EKF::PredictStateAndCovarianceByIMU(crsp_cState filter_state,
                                         crsp_cZFrame frame, crsp_cZImu imu,
                                         spState predicted_state) {
  // 1.
  // this->PredictCovarianceByIMU(filter_state, frame, imu, predicted_state);
  // 2.
  this->PredictStateByIMU(filter_state, frame, imu, predicted_state);
  // 3. 融合状态重置
  // predicted_state->fs_.reset();
  predicted_state->status_ = 0;
}

void EKF::ConfirmInitialCovariance(spState state) {
  auto const &fp = g_config->fuse_cfg().filter_para();
  Eigen::VectorXd P = Eigen::VectorXd::Zero(kNS);
  std::cout << fp.init_att_error_deg().size() << std::endl;

  P.segment<3>(kIdxQvv1) =
      Eigen::Map<Eigen::Vector3d const>(fp.init_att_error_deg().data()) *
      gl_deg;
  P.segment<3>(kIdxVel) =
      Eigen::Map<Eigen::Vector3d const>(fp.init_vel_error_mps().data()) *
      gl_mps;
  P.segment<3>(kIdxPos) =
      Eigen::Map<Eigen::Vector3d const>(fp.init_pos_error_m().data()) * gl_m;
  P.segment<3>(kIdxBg) =
      Eigen::Map<Eigen::Vector3d const>(fp.init_gyr_bias_error_dps().data()) *
      gl_dps;
  P.segment<3>(kIdxBa) =
      Eigen::Map<Eigen::Vector3d const>(fp.init_acc_bias_error_mg().data()) *
      gl_mg;

  // 初始安装角误差
  if (g_config->fuse_cfg().online_estimation_qvv1()) {
    P.segment<3>(kIdxQvv1) =
        Eigen::Map<Eigen::Vector3d const>(fp.init_qvi_error_deg().data()) *
        gl_deg;
  }
  if (g_config->fuse_cfg().online_estimation_ko()) {
    P.segment<4>(kIdxKod) =
        Eigen::Map<Eigen::Vector4d const>(fp.init_kod_error().data());
  }

  state->cov_ = P.cwiseAbs2().asDiagonal();
}

}  // namespace civloc
}  // namespace civ
