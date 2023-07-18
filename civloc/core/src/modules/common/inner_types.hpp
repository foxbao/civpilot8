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
#include <bitset>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "common/interface/logger/log_interface.h"
#include "proto/locator.pb.h"
#include "proto/sensors_setting.pb.h"

namespace civ {
namespace civloc {
#define cr(T) T const &
#define sp(T) std::shared_ptr<T>
#define up(T) std::unique_ptr<T>
#define crsp_c(T) std::shared_ptr<T const> const &
#define sp_c(T) std::shared_ptr<T const>
#define crsp(T) std::shared_ptr<T> const &

#define DEFINE_EXTEND_TYPE(T)                         \
  using cr##T = T const &;                            \
  using sp##T = std::shared_ptr<T>;                   \
  using up##T = std::unique_ptr<T>;                   \
  using crsp_c##T = std::shared_ptr<T const> const &; \
  using sp_c##T = std::shared_ptr<T const>;           \
  using crsp##T = std::shared_ptr<T> const &

using crString = std::string const &;  // std::string const&
using Vec3d = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;

// ! 内部结构体定义 增加类型需要修改 operator<<
enum class ZFrameType {
  None = 0,
  VEHICLE_MODEL = 1,  // 车辆运动约束虚拟帧
  // 虚拟帧
  IMU = 5,        // IMU
  WHEEL_ODO = 6,  // 车轮里程计
  GNSS_POS = 7,   // GNSS 观测值
  GNSS_VEL = 8,   // GNSS 观测值
  GNSS_ALL = 9,   // GNSS 观测值[vel pos dual_antenna]
  //
  CGI610 = 10,
  Cpt7 = 11,
  STATE = 15,
  PERCEPTION = 20,
  SEMANTIC_MAP = 21,
  LINESEGMENT = 25,
  POLE = 26,
  SIGN = 27,
  SEMANTIC_MATCH = 40,
  // LIDAR
  PointCloud = 50,
  LIDAR_MATCH = 51,            // 激光匹配结果
  PointCloudMapFileInfo = 52,  // 地图文件信息
};

struct ZFrame {
  ZFrame() {}
  virtual ~ZFrame() {}
  double t0_, t1_, t2_;  // 产生时刻, 接收时刻，计算完成时刻,s
  ZFrameType type_;      // 数据类型
  std::string channel_name_;  // 数据所在通道
};
DEFINE_EXTEND_TYPE(ZFrame);

struct ZImu : public ZFrame {
  ZImu() { type_ = ZFrameType::IMU; }
  Eigen::Vector3d acc_ = {0, 0, 0};  // 加速度 g
  Eigen::Vector3d gyr_ = {0, 0, 0};  // 角速度 rad/s
  double temper_ = 0;                // 温度 ℃
};
DEFINE_EXTEND_TYPE(ZImu);

enum class GnssStatus : size_t {
  GS_INVALID = 0,  // 无效
  GS_SPP,          // 单点定位
  GS_DGPS,         // 伪距差分
  GS_RESERVE,      // 保留位
  GS_FIXED,        // RTK固定解
  GS_FLOAT         // RTK浮点解
};
inline bool operator==(int32_t left, GnssStatus const &right) {
  return left == static_cast<int32_t>(right);
}
inline bool operator!=(int32_t left, GnssStatus const &right) {
  return !(left == right);
}

struct ZGnss : public ZFrame {
  ZGnss() { type_ = ZFrameType::GNSS_ALL; }
  Eigen::Vector3d pos_ = {0, 0, 0};  // 纬度，经度，大地高 rad,rad,m
  Eigen::Vector3d vel_ = {0, 0, 0};  // ve,vn,vu m/s
  double azi_track_ = 0;             // 方位角 rad
  Eigen::Vector2d dual_antenna_angle_ = {
      0, 0};  // 双天线方位角azimuth rad  pitch rad
  Eigen::Vector2d dual_antenna_std_ = {0, 0};  // 标准差 azimuth pitch (rad rad)
  double speed_ = 0;                           // 地速(水平速度) m/s
  int32_t status_ =
      0;  // GGA解状态, 0-nofix, 1-SPS单点, 2-DGPS伪距差分, 4-RTK, 5-float
  int32_t msg_type_ =
      0;  // message type，1<<1:gga 1<<2:gga+rmc 1<<3:bestpos+bestvel 1<<4:
          // bestpos+bestvel+Dual antenna enable
  int32_t sat_num_ = 0;                  // 卫星数
  double hdop_ = 0;                      // hdop of current epoch[暂未使用]
  double rtk_age_ = 0;                   // 差分龄期(sec)
  Eigen::Vector3d std_pos_ = {0, 0, 0};  // latitude,longitude,altitude (m)
  Eigen::Vector3d std_vel_ = {0, 0, 0};  // ve,vn,vu (m/s)
};
DEFINE_EXTEND_TYPE(ZGnss);

enum class CHC_SYSTEM_STATE {
  Initialing = 0,
  GNSS_ONLY,      // 卫星导航模式
  COMBINATION,    // 组合导航模式
  DEAD_RECKONING  // 行位推算模式
};

enum class CHC_SATELLITE_STATUS {
  NONE = 0,
  SPP,                       // 单点定位定向
  DGPS,                      // 伪距差分定位定向
  DR,                        // 组合推算
  RTK_FIXED,                 // rtk固定解
  RTK_FLOAT,                 // rtk浮点解
  SPP_NO_ORIENTATION,        // 单点定位不定向
  DGPS_NO_ORIENTATION,       // 伪距差分定位不定向
  RTK_FIXED_NO_ORIENTATION,  // rtk固定解定位不定向
  RTK_FLOAT_NO_ORIENTATION,  // rtk浮点解不定向
};

struct ZCnState : public ZFrame {
  ZCnState() { type_ = ZFrameType::CGI610; }
  Eigen::Vector3d gyr_ = {0, 0, 0};  // 角速度 rad/sec 分辨率0.01°/s
  Eigen::Vector3d acc_ = {0, 0, 0};  // 加速度 g 分辨率1e-4g
  CHC_SYSTEM_STATE sys_state_;       // 系统状态
  unsigned int gps_num_sat_used_[2] = {0, 0};  // 主天线,从天线使用的卫星个数
  CHC_SATELLITE_STATUS sat_status_;            // 解状态
  double age_ = 0;                             // 差分龄期 s
  unsigned int gps_num_sats_[2] = {0, 0};  // 主天线,从天线搜索到的卫星个数
  Eigen::Vector3d pos_ = {
      0, 0, 0};  // 大地纬度,大地经度,大地高 (rad,rad,m) 分辨率1e-8°
  Eigen::Vector3d pos_std_ = {0, 0, 0};  // 东北天位置1sigma m 分辨率1e-4m
  Eigen::Vector3d vel_ = {0, 0, 0};  // 东北天 水平速度 m/s, 分辨率 1e-2/s
  Eigen::Vector3d vel_std_ = {0, 0, 0};  // 东北天速度1sigma 分辨率1e-3m/s
  Eigen::Vector3d acc_in_vehicle_ = {0, 0, 0};  // 车体系下的加速度 分辨率1e-4g
  Eigen::Vector3d att_ = {0, 0, 0};  // 姿态 [俯仰,翻滚,航向] rad 分辨率1e-2°
  Eigen::Vector3d att_std_ = {
      0, 0, 0};  // 姿态1sigma [俯仰,翻滚,航向] rad 分辨率1e-4°
  Eigen::Vector3d paltance_in_vehicle_ = {0, 0,
                                          0};  // 车体系下的角速度 分辨率1e-2°/s
};
DEFINE_EXTEND_TYPE(ZCnState);
struct KITTI_RAW {
  int64_t timestamp_us;
  float lat;    // latitude of the oxts-unit (deg)
  float lon;    // longitude of the oxts-unit (deg)
  float alt;    // altitude of the oxts-unit (m)
  float roll;   // roll angle (rad),    0 = level, positive = left side up,
                // range: -pi   .. +pi
  float pitch;  // pitch angle (rad),   0 = level, positive = front down, range:
                // -pi/2 .. +pi/2
  float yaw;    // heading (rad),       0 = east,  positive = counter clockwise,
                // range: -pi   .. +pi
  float vn;     // velocity towards north (m/s)
  float ve;     // velocity towards east (m/s)
  float vf;     // forward velocity, i.e. parallel to earth-surface (m/s)
  float vl;     // forward velocity, i.e. parallel to earth-surface (m/s)
  float vu;     // upward velocity, i.e. perpendicular to earth-surface (m/s)
  float ax;     // acceleration in x, i.e. in direction of vehicle front (m/s^2)
  float ay;     // acceleration in y, i.e. in direction of vehicle left (m/s^2)
  float az;     // acceleration in z, i.e. in direction of vehicle top (m/s^2)
  float af;     // forward acceleration (m/s^2)
  float al;     // leftward acceleration (m/s^2)
  float au;     // upward acceleration (m/s^2)
  float wx;     // angular rate around x (rad/s)
  float wy;     // angular rate around y (rad/s)
  float wz;     // angular rate around z (rad/s)
  float wf;     // angular rate around forward axis (rad/s)
  float wl;     // angular rate around leftward axis (rad/s)
  float wu;     // angular rate around upward axis (rad/s)
  float pos_accuracy;  // velocity accuracy (north/east in m)
  float vel_accuracy;  // velocity accuracy (north/east in m/s)
  float navstat;       // navigation status (see navstat_to_string)
  float numsats;       // number of satellites tracked by primary GPS receiver
  float posmode;       // position mode of primary GPS receiver (see
                       // gps_mode_to_string)
  float velmode;       // velocity mode of primary GPS receiver (see
                       // gps_mode_to_string)
  float orimode;       // orientation mode of primary GPS receiver (see
                       // gps_mode_to_string)

  void pos2rad() {
    lat = lat * M_PI / 180;
    lon = lon * M_PI / 180;
  }
};
DEFINE_EXTEND_TYPE(KITTI_RAW);

typedef enum class VehicleStateIndex : int {
  // UNKNOWN = 0,
  // ATT 姿态相关 [爬坡 下坡 左翻滚 右翻滚]
  // VEL 速度相关
  VEL_STATIC = 8,
  VEL_LOW_SPEED = 9,
  VEL_HIGH_SPEED = 10,
  // ACC 加速度相关
  ACC_UNIFORM = 16,
  ACC_ACCELERATE = 17,
  ACC_RAPID_ACCELERATE = 18,
  ACC_DECELERATE = 19,
  ACC_RAPID_DECELERATE = 20,
  // PAL 角速度相关 [抬头机动 低头机动]
} VSI;

template <size_t N>
struct VehicleState : public std::bitset<N> {
  bool get_is_static() const {
    return this->test(static_cast<int>(VSI::VEL_STATIC));
  }
  void set_static() { this->set(static_cast<int>(VSI::VEL_STATIC), true); }
};

struct ZVehicleModel : public ZFrame {
  ZVehicleModel() {
    type_ = ZFrameType::VEHICLE_MODEL;
    channel_name_ = "ZFrameType::VEHICLE_MODEL";
  }

  VehicleState<32> vs_;          // 车辆状态定义
  bool is_first_static = false;  // 第一次静止
  // 静止加速度计,陀螺噪声
  ZImu mean_imu_;  // 静止的IMU状态,没有扣除bias
  Eigen::Vector3d gyr_std_ = {0, 0, 0};
  Eigen::Vector3d acc_std_ = {0, 0, 0};
};
DEFINE_EXTEND_TYPE(ZVehicleModel);

struct State;
DEFINE_EXTEND_TYPE(State);

enum class ZIntrinsicParameterType { Imu = 0, Camera };
struct IntrinsicParameter {
  IntrinsicParameter() {}
  virtual ~IntrinsicParameter() {}
  ZIntrinsicParameterType type_;
};
using spIntrinsicParameter = std::shared_ptr<IntrinsicParameter>;

// 相机内参
struct IntrinsicCamPara : public IntrinsicParameter {
  Eigen::Matrix3d K_;  // 内参矩阵
  std::vector<double> distortion_;
};
DEFINE_EXTEND_TYPE(IntrinsicCamPara);

// IMU内参
struct IntrinsicImuPara : public IntrinsicParameter {
  Eigen::Vector3d ba_ = Eigen::Vector3d::Zero();  // 加表零偏 m/s/s
  Eigen::Vector3d bg_ = Eigen::Vector3d::Zero();  // 陀螺零偏 rad/s
  Eigen::Matrix3d acc_skew_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d gyr_skew_ = Eigen::Matrix3d::Identity();
  // ....
};
DEFINE_EXTEND_TYPE(IntrinsicImuPara);

enum class MeasurementType;
template <size_t N>
struct FilterState : public std::bitset<N> {
  // std::bitset<32> fs_ = {0};  // vehicle state
  bool get_state(MeasurementType const &mt) const {
    return this->test(static_cast<int>(mt));
  }
  void set_state(MeasurementType const &mt) {
    this->set(static_cast<int>(mt), true);
  }
};

typedef struct StateIndex {
  int qnn1_ = 0, v_ = 0, p_ = 0, bg_ = 0, ba_ = 0, gz_scale_ = 0, qvv1_ = 0;
  int tia_ = 0, tva_ = 0, tdg_ = 0;            // GNSS外参
  int kod_ = 0, tdo_ = 0;                      // 轮速外参
  int qcc1_ = 0, tic_ = 0, tdc_ = 0, tr_ = 0;  // camera外参
  int fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;      // camera内参
  int NS_ = 0;                                 // 总状态维数
} SI;

// 存储单个时刻的状态,主要指IMU状态，enu坐标系为当前车所在位置建立的东北天坐标系，车体系为后轴中心点的右前上方向
struct State : public ZFrame {
 public:
  State() { type_ = ZFrameType::STATE; }
  Eigen::Vector3d pos_ = {0, 0, 0};  // 纬度，经度，高度 [rad rad m]
  Eigen::Vector3d vel_ = {0, 0, 0};  // 当前IMU所在位置的enu系速度 [m/s]
  Eigen::Quaterniond qua_ =
      Eigen::Quaterniond::Identity();  // 当前IMU所在位置的enu系姿态, qnv
  Eigen::Vector3d wv_ = {0, 0, 0};     // imu在车体坐标系下的角速度
  Eigen::Vector3d fv_ = {0, 0, 0};
  double mileage_ = 0;
  Eigen::Vector3d std_pos_ = {0, 0, 0};  // enu位置标准差 [m]
  Eigen::Vector3d std_vel_ = {0, 0, 0};  // enu速度标准差 [m/s]
  Eigen::Vector3d std_att_ = {0, 0, 0};  // enu姿态标准差 [rad]
  // 内外参数 extrinsic intrinsic

  IntrinsicImuPara imu_in_para_;  // imu内参
  IntrinsicCamPara cam_in_para_;  // 相机内参
  Eigen::Vector4d odo_in_para_;   // 轮速内参

  // 当前外参均为相对于悬挂的RT
  std::map<std::string, Eigen::Isometry3d> ex_para_;
  // // 状态方差
  Eigen::MatrixXd cov_;    // 状态方差
  static StateIndex idx_;  // 状态序号

  //! 车辆状态，融合获得ZVehivleModel数据帧时更新,预测器输出该结果可能会存在延迟
  VehicleState<32> vs_;
  // 状态每次需要重置
  int status_ = 0;      // 状态定义,暂时与cgi610的定位状态一致
  FilterState<64> fs_;  //! 滤波状态

 public:
  Eigen::Matrix3d const c_vi() const { return ex_para_.at("imu").linear(); }
  Eigen::Matrix3d const c_nv() const {
    return qua_ * T_vi().linear().transpose();
  }
  Eigen::Isometry3d const T_vi() const { return ex_para_.at("imu"); }
  Eigen::Isometry3d const T_vg() const { return ex_para_.at("gnss"); }
  Eigen::Isometry3d const T_vl() const { return ex_para_.at("lidar"); }
  Eigen::Isometry3d const T_vc() const { return ex_para_.at("perc"); }
  Eigen::Vector4d const odo_ko() const { return odo_in_para_; }
};

enum class MeasurementType {
  None = 0,
  // imu
  // gnss
  GNSS_DUAL_ANTENNA = 0x08,
  GNSS_VEL = 0x08 + 1,
  GNSS_POS = 0x08 + 2,
  // wheel odo
  // wheel odo
  WHEEL_ODO_REAR = 0x0f,
  WHEEL_ODO_FRONT = 0x0f + 1,
  WHEEL_ODO_REAR_DIFF = 0x0f + 2,
  WHEEL_ODO_FRONT_DIFF = 0x0f + 3,
  // vehicle model
  VEHICLE_MODEL_NHC = 0x10,
  VEHICLE_MODEL_ZUPT = 0x10 + 1,
  VEHICLE_MODEL_ZGBC = 0x10 + 2,
  VEHICLE_MODEL_ZPC = 0x10 + 3,
  VEHICLE_MODEL_ZAC = 0x10 + 4,
  //
  VISION_SEMANTIC_MATCH = 0x18,
  // lidar match
  LIDAR_MATCH = 0x20,
  LIDAR_POS = 0x21,
  LIDAR_ORI = 0x22
};

}  // namespace civloc
}  // namespace civ
