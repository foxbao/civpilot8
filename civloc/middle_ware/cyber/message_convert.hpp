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
#include "message/drivers/gnss/proto/raw_gnss.pb.h"
#include "message/drivers/imu/proto/imu.pb.h"
#include "message/localization/proto/localization.pb.h"
#include "modules/common/inner_types.hpp"

namespace civ {
namespace civloc {

using IMUProto = civ::drivers::imu::CorrectedImu;
using GNSSProto = civ::drivers::rawgnss::GNGGA;
using LocalizationEstimate =
    civ::localization::LocalizationEstimate;  // 对外输出的定位结果
DEFINE_EXTEND_TYPE(IMUProto);
DEFINE_EXTEND_TYPE(GNSSProto);

sp_cZImu Convert(crString channel_name, crsp_cKITTI_RAW frame);
sp_cZImu Convert(crString channel_name, crsp_cIMUProto frame);
sp_cZGnss Convert(crString channel_name, crsp_cGNSSProto frame);
LocalizationEstimate Convert(crsp_cState frame);  // 定位输出

sp_cZCnState Convert_cgi610(crString channel_name,
                            crsp_cGNSSProto frame);  // cgi610组合解
sp_cZCnState Convert_cgi610(crString channel_name,
                            crsp_cKITTI_RAW frame);  // cgi610组合解
sp_cZGnss Convert_gnss(crString channel_name, crsp_cKITTI_RAW frame);

spState Convert_state(crsp_cKITTI_RAW frame);
}  // namespace civloc
}  // namespace civ
