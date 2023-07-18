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

#include "modules/common/converter.hpp"
#include <memory>
// #include "modules/common/earth.hpp"
#include "common/coordinate_transform/earth.hpp"

namespace civ {
namespace civloc {

spState Convert(crsp_cZCnState frame) {
  spState result = std::make_shared<State>();

  result->t0_ = frame->t0_;
  result->t1_ = frame->t1_;
  result->pos_ = frame->pos_;
  result->vel_ = frame->vel_;
  result->qua_ =common::coord_transform::a2qua(frame->att_);
  
  result->std_pos_ = frame->pos_std_;
  result->std_vel_ = frame->vel_std_;
  result->std_att_ = frame->att_std_;
  //
  result->wv_ = frame->paltance_in_vehicle_;
  result->fv_ = frame->acc_in_vehicle_;
  result->status_ = static_cast<int>(frame->sat_status_);

  return result;
}

}  // namespace civloc
}  // namespace civpilot
