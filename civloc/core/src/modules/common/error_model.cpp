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

#include <memory>
#include "modules/common/error_model.hpp"

namespace civ {
namespace civloc {

spZImu Compenstate(ZImu const& raw, IntrinsicImuPara const& in_para) {
  spZImu imu = std::make_shared<ZImu>(raw);
  // imu->acc_ = in_para.acc_skew_ * (raw.acc_ - in_para.ba_ / gl_g0);
  imu->acc_ = in_para.acc_skew_ * (raw.acc_ - in_para.ba_);
  imu->gyr_ = in_para.gyr_skew_ * (raw.gyr_ - in_para.bg_);
  return imu;
}

}  // namespace civloc
}  // namespace civ
