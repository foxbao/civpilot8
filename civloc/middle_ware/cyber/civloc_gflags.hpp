/******************************************************************************
 * Copyright 2017 The zhito Authors. All Rights Reserved.
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

/**
 * @file localization_gflags.h
 * @brief The gflags used by localization module
 */

#pragma once

#include "gflags/gflags.h"
#include "common/configs/config_gflags.h"

namespace civ {
namespace civloc {
// 传感器,locator配置文件
DECLARE_string(sensors_setting_path);
DECLARE_string(locator_setting_path);
}  // namespace civloc

}  // namespace civ
