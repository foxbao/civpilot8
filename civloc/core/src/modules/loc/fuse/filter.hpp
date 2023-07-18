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
#include <map>
#include <memory>
#include <string>

// #include "modules/common/earth.hpp"
#include "common/coordinate_transform/earth.hpp"
#include "modules/common/inner_types.hpp"
#include "modules/common/time_system.hpp"
#include "modules/common/utils.hpp"
namespace civ {
namespace civloc {


// 各类融合方法基类
class Filter {
 public:
  virtual ~Filter() {}
};
DEFINE_EXTEND_TYPE(Filter);

class EKF : public Filter {
 public:
  EKF();
  // 确定滤波器初始方差
  void ConfirmInitialCovariance(spState state);
  // 滤波状态更新 IMU为进行过误差修正后的数据
  void PredictStateAndCovarianceByIMU(crsp_cState filter_state,
                                      crsp_cZFrame frame, crsp_cZImu imu,
                                      spState predicted_state);

 private:
  void PredictCovarianceByIMU(crsp_cState filter_state, crsp_cZFrame frame,
                              crsp_cZImu imu, spState predicted_state);
  void PredictStateByIMU(crsp_cState filter_state, crsp_cZFrame frame,
                         crsp_cZImu imu, spState predicted_state);
  // void PredictStateByIMU();

 public:
  Eigen::MatrixXd &H() { return H_; }
  Eigen::MatrixXd &ResetH(size_t const row, size_t const col);
  Eigen::MatrixXd &R() { return R_; }
  Eigen::VectorXd &Z() { return Z_; }

  template <typename scalar, int row, int col>
  void set_dpos_dfai(Eigen::Matrix<scalar, row, col> const &mat) {
    H_.block<row, col>(0, col) = mat;
  }

 public:
  // 1  计算卡方值
  double ComputeChiSquare(crsp_cState state);
  // 2 计算修正量
  Eigen::VectorXd ComputeDeltaX(crsp_cState state);
  // 3 更新状态方差
  void UpdateP(spState state);
  // 4 反馈
  void CompensateDeltaX(spState state, Eigen::VectorXd const &dx);

 private:
  Eigen::MatrixXd H_;
  Eigen::MatrixXd R_;
  Eigen::VectorXd Z_;
  Eigen::VectorXd dx_;
  Eigen::VectorXd qt_;  // 过程噪声
  Eigen::MatrixXd K_;

 private:
  bool DetermineSensors();
  void DetermineIndex();
  void ConfirmProgressNoise();

 private:
  Eigen::MatrixXd trans_phi_;          // P阵延迟更新使用
  Eigen::MatrixXd trans_noise_;        //
  Eigen::MatrixXd innov_var_inverse_;  // 新息方差矩阵
};

DEFINE_EXTEND_TYPE(EKF);
extern spEKF g_eskf;
}  // namespace civloc
}  // namespace civ
