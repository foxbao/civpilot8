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

#include "middle_ware/cyber/civloc_component.hpp"
#include <deque>
#include <vector>
#include "common/coordinate_transform/earth.hpp"
#include "common/coordinate_transform/LocalCartesian_util.h"
#include "common/gnss_third_party/rtklib.h"
#include "common/interface/file/file.h"
#include "cyber/proto/dag_conf.pb.h"
#include "interface/locator.h"
#include "middle_ware/cyber/civloc_gflags.hpp"
#include "middle_ware/cyber/message_convert.hpp"
#include "middle_ware/cyber/tool/preprocessor.hpp"
namespace civ {
namespace civloc {

CivLocComponent::CivLocComponent(std::string const &dag_file_path) {
  using apollo::cyber::proto::DagConfig;
  node_ = apollo::cyber::CreateNode("civloc");
  DagConfig dag_config;
  apollo::cyber::common::GetProtoFromFile(dag_file_path, &dag_config);
  LoadConfigFiles(dag_config.module_config(0).components(0).config());
  InitCore();
  InitIO();
  fused_output_data_path_ =
      "/home/baojiali/Downloads/civpilot/data/fused_result.txt";
  fused_output_data_ = std::ofstream(fused_output_data_path_);
  if (!fused_output_data_.is_open()) {
    std::cout << "fuse file open fail" << std::endl;
    return;
  }

  fused_output_enu_data_path_ =
      "/home/baojiali/Downloads/civpilot/data/fused_enu_result.txt";
  fused_output_enu_data_ = std::ofstream(fused_output_enu_data_path_);
  if (!fused_output_enu_data_.is_open()) {
    std::cout << "fuse file open fail" << std::endl;
    return;
  }
  gnss_output_data_path_ =
      "/home/baojiali/Downloads/civpilot/data/gnss_result.txt";
  gnss_output_data_ = std::ofstream(gnss_output_data_path_);
  if (!gnss_output_data_.is_open()) {
    std::cout << "gnss file open fail" << std::endl;
    return;
  }
}

CivLocComponent::~CivLocComponent() { fused_output_data_.close(); }
// CivLocComponent::CivLocComponent(std::string const &config_file_path,
//                                  std::string const &locator_setting_path,
//                                  std::string const &sensors_setting_path) {
//   // config_file_path_ = config_file_path;
//   // locator_setting_pth_ = locator_setting_path;
//   // sensors_setting_pth_ = sensors_setting_path;

//   // fused_output_data_path_ =
//   // "/home/baojiali/Downloads/civpilot/loc/raw_imu.txt"; output_data_ =
//   // std::ofstream(fused_output_data_path_, std::ios::app); sp_imgprocessor_ =
//   // std::make_shared<IMGPROCESSOR>();
// }

bool CivLocComponent::Init() {
  InitCore();
  InitIO();
  return true;
}

bool CivLocComponent::InitCore() {
  AINFO << config_file_path_ << " zloc has been started!!!";
  using civ::common::file::GetProtoFromFile;
  cfg_ = std::make_shared<CivLocConfig>();

  ACHECK(GetProtoFromFile(config_file_path_, cfg_.get()))

      << "load from file failed, path = " << config_file_path_;
  AINFO << "FLAGS_locator_setting_path = " << FLAGS_locator_setting_path;
  AINFO << "FLAGS_sensors_setting_path = " << FLAGS_sensors_setting_path;
  locator_ = std::make_shared<Locator>(FLAGS_locator_setting_path,
                                       FLAGS_sensors_setting_path);
  if (locator_) {
    AWARN << "ZLocComponent (locator_) init done.";
  }

  double acc_noise, gyro_noise, acc_bias_noise, gyro_bias_noise;
  double I_p_Gps_x, I_p_Gps_y, I_p_Gps_z;

  acc_noise = 1e-2;
  gyro_noise = 1e-4;
  acc_bias_noise = 1e-6;
  gyro_bias_noise = 1e-8;
  I_p_Gps_x = 0;
  I_p_Gps_y = 0;
  I_p_Gps_z = 0;
  const Eigen::Vector3d I_p_Gps(I_p_Gps_x, I_p_Gps_y, I_p_Gps_z);
  imu_gps_localizer_ptr_ = std::make_shared<ImuGpsLocalizer>(
      acc_noise, gyro_bias_noise, acc_bias_noise, gyro_bias_noise, I_p_Gps);

  return true;
}

void CivLocComponent::ReadKittiData(crString data_folder,
                                    crString timestamps_file) {
  // sp_imgprocessor->PlotPositionCovariance();
  // DataReader
  spDataReader sp_dataReader = std::make_shared<DataReader>();
  std::vector<int64_t> timestamps_us;
  std::vector<KITTI_RAW> kitti_data;
  sp_dataReader->ReadKittiTimeStamps(timestamps_file, timestamps_us);
  sp_dataReader->ReadKittiRawData(data_folder, kitti_data);

  spState mecha_current_state = std::shared_ptr<State>();

  std::deque<sp_cZImu> imu_deque_delay_one;
  for (size_t i = 0; i < kitti_data.size(); i++) {
    // generate kitti data
    kitti_data[i].pos2rad();
    kitti_data[i].timestamp_us = timestamps_us[i];
    spKITTI_RAW kitti_frame = std::make_shared<KITTI_RAW>(kitti_data[i]);
    // AINFO << "data time:" << std::setprecision(12) <<
    // double(kitti_data[i].timestamp_us) / 1000000;
    std::cout << "--------data time---------:" << std::setprecision(12)
              << double(kitti_frame->timestamp_us) / 1000000 << std::endl;

    // pure mechanization
    if (i == 0) {
      mecha_current_state = Convert_state(kitti_frame);
    } else {
      auto const &imu = Convert("rawimu", kitti_frame);
      if (imu_deque_delay_one.empty()) {
        imu_deque_delay_one.push_back(imu);
      } else {
        imu_deque_delay_one.push_back(imu);
        auto imu_delay = imu_deque_delay_one.front();
        imu_deque_delay_one.pop_front();
        // AINFO << "imu time" << std::setprecision(12) << imu->t0_;

        // std::cout << "pos ori Mec:" << std::setprecision(12) <<
        // mecha_current_state->pos_.transpose() << std::endl;

        mecha_current_state =
            PredictStateMecanization(mecha_current_state, imu_delay);

        if (mecha_current_state && i % 10 == 9) {
          // sp_imgprocessor_->PlotSaveStateLLH(mecha_current_state,
          //                                    "_imu_mecha ");
        }
      }
    }

    // use filter
    if (i == 0) {
      // 第一帧先虚拟成CGI610的帧,
      sp_cZCnState cgi610 = Convert_cgi610("cgi610", kitti_frame);
      // cgi610->age_=100;//civ::civloc::CHC_SATELLITE_STATUS::RTK_FIXED;
      locator_->ProcessCGI610Frame(cgi610);
    } else {
      // The kitti data contains llh and imu together, therefore we generate
      // GNSS and IMU together generate GNSS from KITTI 1 out of 10 frames
      if (i % 10 == 9) {
        auto const &gnss_frame = Convert_gnss("rawgnss", kitti_frame);
        locator_->ProcessGnss(gnss_frame);
      }

      auto const &imu_frame = Convert("rawimu", kitti_frame);

      auto const &state = locator_->ProcessIMU(imu_frame);
      // if(state)
      // {
      //     sp_imgprocessor->PlotSaveStateLLH(state);
      // }

      locator_->FuseTaskSingle();
      int aaa = 1;
    }

    // spState aligned_state = initializer_->get_aligned_state();
    // fuse_processor_->ConfirmInitialState(aligned_state);
    // fuse_processor_->PredictAndDoFuse(Convert(frame));
    // ProcessIMU();
  }
}

spState CivLocComponent::PredictStateMecanization(spState filter_state,
                                                  sp_cZImu imu) {
  // std::cout<<"filter_state time in mec"<<filter_state->t0_<<std::endl;
  // std::cout<<"gyro time in mec"<<imu->t0_<<std::endl;
  using namespace civ::common::coord_transform;
  spState predicted_state = std::make_shared<State>();

  double dt = imu->t0_ - filter_state->t0_;
  // std::cout<<"dt mec:" << std::setprecision(12) << dt<<std::endl;

  Eigen::Vector3d wnie = Earth::GetWnie(filter_state->pos_),
                  wnen = Earth::GetWnen(filter_state->pos_, filter_state->vel_);
  // Vector3d const vel_inc = imu->acc_ * gl_g0 * dt; // 可以使用差值结果
  Eigen::Vector3d const vel_inc = imu->acc_ * dt;  // 可以使用差值结果

  Eigen::Vector3d const ang_inc =
      imu->gyr_ * dt;  // 可以使用差值结果  //z轴结果进行了调整

  // 速度更新
  Eigen::Vector3d wnin = wnie + wnen;
  Eigen::Quaterniond q_nn1 = rv2q(-wnin * dt);
  Eigen::Vector3d dvn = q_nn1 * filter_state->qua_ * vel_inc;
  Eigen::Vector3d dv_gcor = (-(wnie + wnin).cross(filter_state->vel_) +
                             Earth::GetGn(filter_state->pos_)) *
                            dt;  // 重力及哥氏加速度
  Eigen::Vector3d vel_last = filter_state->vel_;
  predicted_state->vel_ = filter_state->vel_ + (dvn + dv_gcor);

  // 位置更新
  Eigen::Matrix3d cen = Earth::Pos2Cne(filter_state->pos_).transpose();
  predicted_state->pos_ =
      Earth::ECEF2LLH(Earth::LLH2ECEF(filter_state->pos_) +
                      cen * (vel_last + predicted_state->vel_) / 2 * dt);

  // 姿态更新

  predicted_state->qua_ = q_nn1 * filter_state->qua_ * rv2q(ang_inc);
  predicted_state->qua_.normalize();

  // 时间更新
  predicted_state->t0_ = imu->t0_;
  return predicted_state;
}

bool CivLocComponent::InitIO() {
  using apollo::cyber::ReaderConfig;
  ReaderConfig reader_config;

  auto raw_imu = cfg_->raw_wheeltec_imu();

  reader_config.channel_name = raw_imu.name();
  reader_config.pending_queue_size = raw_imu.rate_hz() * 2;

  imu_listener_=node_->CreateReader<IMUProto>(
      reader_config, [this](auto const &msg) { this->OnGetRawImuFrame(msg); });

  gnss_listener_=node_->CreateReader<GNSSProto>(
      "/GNGGA",[this](auto const &msg){
        this->OnGetGNGGAFrame(msg);
      }
  );

  msfr_writer_ =
      node_->CreateWriter<LocalizationEstimate>("LocalizationEstimation");
  return true;
}

void CivLocComponent::OnGetRawImuFrame(crsp_cIMUProto msg) {
  // the state predicted from last good state
  using namespace civ::common::coord_transform;
  ImuGpsLocalization::ImuDataPtr imu_data_ptr =
      std::make_shared<ImuGpsLocalization::ImuData>();
  imu_data_ptr->timestamp = msg->measurement_time();
  imu_data_ptr->acc << msg->linear_acceleration().y(),
      msg->linear_acceleration().x(), -msg->linear_acceleration().z();

  imu_data_ptr->gyro << msg->angular_velocity().y(),
      msg->angular_velocity().x(), -msg->angular_velocity().z();

  ImuGpsLocalization::State fused_state;
  const bool ok =
      imu_gps_localizer_ptr_->ProcessImuData(imu_data_ptr, &fused_state);
  if (!ok) {
    return;
  }
  spState state = std::make_shared<State>();
  state->t0_ = fused_state.timestamp;
  state->t1_ = fused_state.timestamp;

  state->pos_ = Eigen::Vector3d{fused_state.lla(0) * D2R,
                                fused_state.lla(1) * D2R, fused_state.lla(2)};
  state->cov_ = fused_state.cov;
  state->vel_ = fused_state.G_v_I;
  state->qua_ = fused_state.G_R_I;


  Eigen::Vector3d pos_enu;
  // civ::common::coord_transform::ConvertVectorInA
  // ConvertENUToLLA(Earth::GetOrigin(), state_.G_p_I, &(state_.lla));
  ConvertLLAToENU(Earth::GetOrigin(),
                  Eigen::Vector3d{state->pos_(0) * R2D, state->pos_(1) * R2D,
                                  state->pos_(2)},
                  &pos_enu);

  fused_output_data_ << std::setprecision(17) << state->t0_ << ","
                     << state->pos_(0) << "," << state->pos_(1) << ","
                     << state->pos_(2) << std::endl;

  fused_output_enu_data_ << std::setprecision(17) << state->t0_ << ","
                     << pos_enu(0) << "," << pos_enu(1) << ","
                     << pos_enu(2) << std::endl;
  msfr_writer_->Write(Convert(state));

  //   imu_data_ptr->timestamp = imu_msg_ptr->header.stamp.toSec();
  // imu_data_ptr->acc << imu_msg_ptr->linear_acceleration.x,
  //                      imu_msg_ptr->linear_acceleration.y,
  //                      imu_msg_ptr->linear_acceleration.z;
  // imu_data_ptr->gyro << imu_msg_ptr->angular_velocity.x,
  //                       imu_msg_ptr->angular_velocity.y,
  //                       imu_msg_ptr->angular_velocity.z;

  // auto const &state =
  //     locator_->ProcessIMU(Convert(cfg_->raw_wheeltec_imu().name(), msg));
  // locator_->FuseTaskSingle();
  // RETURN_IF_NULLPTR(state);
}

void CivLocComponent::OnGetGNGGAFrame(crsp_cGNSSProto msg) {
  ImuGpsLocalization::GpsPositionDataPtr gps_data_ptr =
      std::make_shared<ImuGpsLocalization::GpsPositionData>();
  gps_data_ptr->timestamp = msg->utc_in_seconds();
  gps_data_ptr->lla << msg->latitude_deg(), msg->longitude_deg(), msg->alt();
  Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
  cov(0, 0) = msg->hdop() * msg->hdop();
  cov(1, 1) = msg->hdop() * msg->hdop();
  cov(2, 2) = msg->hdop() * msg->hdop();
  gps_data_ptr->cov = cov;
  imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);

  gnss_output_data_ << std::setprecision(15) << gps_data_ptr->timestamp << ","
                    << gps_data_ptr->lla(0) * D2R << ","
                    << gps_data_ptr->lla(1) * D2R << "," << gps_data_ptr->lla(2)
                    << std::endl;
  // if (!gngga_test_sent) {
  //   imu_gps_localizer_ptr_->ProcessGpsPositionData(gps_data_ptr);
  //   gngga_test_sent = true;
  // }

  // if (!gngga_test_sent) {
  //   locator_->ProcessCGI610Frame(Convert_cgi610("cgi610", msg));
  //   std::cout << std::setprecision(15)
  //             << "getting cgi610:" << msg->utc_in_seconds() << std::endl;
  //   gngga_test_sent = true;
  // }
  // locator_->ProcessGnss(Convert("/GNGGA", msg));
}
void CivLocComponent::set_input_data_dir(std::string const &path) {
  locator_->set_input_data_dir(path);
}

}  // namespace civloc
}  // namespace civ
