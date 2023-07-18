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

#include "modules/detect/detector.hpp"
#include <fstream>
#include <memory>
#include <string>
// #include "modules/common/units.hpp"
#include "common/coordinate_transform/units.hpp"
#include "modules/common/utils.hpp"
#include "modules/message_cache/cacher.hpp"

namespace civ {
namespace civloc {

// 根据最新的IMU以及cacher数据,进行车辆状态检测
sp_cZVehicleModel Detector::Detect(crsp_cZImu frame) {
  // 10hz触发
  size_t const t_ms = s2ms(frame->t0_);
  if (t_ms % 100 != 0) {
    return nullptr;
  }
  // 生成车辆模型数据帧
  vm_ = std::make_shared<ZVehicleModel>();
  vm_->t0_ = frame->t0_;
  // 静止检测, 连续10次静止才算静止
  if (StaticCheck()) {
    AINFO << UnixTimeString(vm_->t0_) << " current is static, static index is "
          << static_index;
  }

  return vm_;
}

bool Detector::StaticCheck() {
  using namespace civ::common::coord_transform;
  bool result = false;
  // 使用IMU进行静态检测
  auto const& detect_cfg = g_config->detect_cfg();
  auto const& imus =
      g_cacher->getMainImuInRange(vm_->t0_, detect_cfg.sample_range_time());
  int count = imus.size();

  // 计算各个检测指标
  Eigen::Vector3d acc_i = {0, 0, 0}, acc_i2 = {0, 0, 0};
  Eigen::Vector3d gyr_i = {0, 0, 0}, gyr_i2 = {0, 0, 0};
  double gyr_norm_i = 0, gyr_norm_i2 = 0;
  double acc_norm_i = 0, acc_norm_i2 = 0;
  for (auto it = imus.begin(); it != imus.end(); ++it) {
    auto const& frame = *it;
    acc_i += frame->acc_, gyr_i += frame->gyr_;
    acc_i2 += Eigen::Vector3d(frame->acc_.array().pow(2)),
        gyr_i2 += Eigen::Vector3d(frame->gyr_.array().pow(2));
    gyr_norm_i += frame->gyr_.norm(), acc_norm_i += frame->acc_.norm();
    gyr_norm_i2 += pow(frame->gyr_.norm(), 2),
        acc_norm_i2 += pow(frame->acc_.norm(), 2);
  }
  vm_->mean_imu_.gyr_ = gyr_i / count;
  vm_->gyr_std_ =
      (gyr_i2 / count - Eigen::Vector3d((gyr_i / count).array().square()))
          .array()
          .sqrt();

  // 静止判断
  auto const& imu_para = g_sensor_config->imu(0);
  auto const& gyr_para = imu_para.gyr();
  auto const& acc_para = imu_para.acc();
  double gyr_noise =
      sqrt(pow(gyr_para.output_noise(0), 2) + pow(gyr_para.output_noise(1), 2) +
           pow(gyr_para.output_noise(2), 2));
  double acc_noise =
      sqrt(pow(acc_para.output_noise(0), 2) + pow(acc_para.output_noise(1), 2) +
           pow(acc_para.output_noise(2), 2));
  double const std[2] = {
      acc_noise * 1e-6,
      gyr_noise * gl_deg};  // IMU三轴合成加速度和角速度测量噪声的方差
  double gyr_mean = gyr_norm_i / count;
  double acc_mean = acc_norm_i / count;
  gyr_stat[0] = gyr_mean;
  gyr_stat[1] = sqrt((gyr_norm_i2 - count * gyr_mean * gyr_mean) / (count - 1));
  gyr_stat[2] = gyr_norm_i2 / (count * std[1] * std[1]);
  acc_stat[0] =
      (acc_norm_i2 - 2 * count * acc_mean + count) / (count * std[0] * std[0]);
  acc_stat[1] =
      (acc_norm_i2 - count * acc_mean * acc_mean) / (count * std[0] * std[0]);
  acc_stat[2] =
      (acc_norm_i2 - 2 * count * acc_mean + count) / (count * std[0] * std[0]) +
      gyr_norm_i2 / (count * std[1] * std[1]);
  if (gyr_stat[0] <= detect_cfg.zero_velocity_detector_threshold(0) * gl_deg) {
    result = true;
  }
  // auto const& gyr_mean = vm_->mean_imu_.gyr_;
  // double const limit[3] = {0.15_dps, 0.1_dps, 0.05_dps};
  // if (gyr_mean[0] <= limit[0] && gyr_mean[1] <= limit[1] && gyr_mean[2] <=
  // limit[2]) {
  //   result = true;
  // }
  // AINFO << "gyr_mean = " << gyr_mean.transpose() / gl_dps << " gyr_std = " <<
  // vm_->gyr_std_.transpose() / gl_dps << " result " << result;

  // static std::ofstream out("/zhito/data/0708/static.txt");
  // out << std::fixed << std::setprecision(6) << UnixTimeString(vm_->t0_) << "
  // " << gyr_mean.transpose() / gl_dps <<" "
  //     << gyr_std.transpose() / gl_dps << std::endl;

  // 本次静止
  if (result) {
    ++static_index;
    // std::cout << " *********timestamp***********" << UnixTimeString(vm_->t0_)
    // << std::endl; std::cout << " *********static_index***********" <<
    // static_index << std::endl;
    int static_continuous_number = detect_cfg.static_continuous_number();
    if (static_index >= static_continuous_number) {
      vm_->vs_.set_static();
      vm_->is_first_static =
          (static_index == static_continuous_number);  // 记录首次静止状态
    }
  } else {
    static_index = 0;
  }

  // CGI610速度和各个检测指标结果离线保存
  // AnalyzeDetectResult(vm_);

  return result;
}

void Detector::AnalyzeDetectResult(sp_cZVehicleModel vm_frame) {
  auto cgi610_cacher = g_cacher->GetNamedBuff2()["/zhito/sensor/gnss/cgi610"];

  if (cgi610_cacher.find(s2ms(vm_frame->t0_)) == cgi610_cacher.end()) {
    return;
  }
  auto const& cgi610_frame = std::dynamic_pointer_cast<ZCnState const>(
      cgi610_cacher[s2ms(vm_frame->t0_)]);

  std::string record_path = g_config->io_cfg().data_dir();
  int pos = record_path.find_last_of('/');
  std::string record_id = record_path.substr(pos + 1, record_path.length());
  std::string output_gt_data = "/zhito/data/" + record_id + "/groundtruth.txt";
  std::string output_rst_data =
      "/zhito/data/" + record_id + "/detect_result.txt";

  std::ofstream gt_data(output_gt_data, std::ios::app);
  if (gt_data.is_open()) {
    gt_data << std::fixed << std::setprecision(3) << cgi610_frame->t0_ << "  "
            << cgi610_frame->t1_ << std::setprecision(10) << "  "
            << cgi610_frame->vel_(0) << "  " << cgi610_frame->vel_(1) << "  "
            << cgi610_frame->vel_(2) << std::endl;
    gt_data.close();
  }

  std::ofstream rst_data(output_rst_data, std::ios::app);
  if (rst_data.is_open()) {
    rst_data << std::fixed << std::setprecision(3) << vm_frame->t0_
             << std::setprecision(10) << "  " << gyr_stat[0] << "  "
             << gyr_stat[1] << "  " << gyr_stat[2] << "  " << acc_stat[0]
             << "  " << acc_stat[1] << "  " << acc_stat[2] << "  " << std::endl;
    rst_data.close();
  }
}

}  // namespace civloc
}  // namespace civ
