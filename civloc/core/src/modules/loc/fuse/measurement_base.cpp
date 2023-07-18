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

#include "modules/loc/fuse/measurement_base.hpp"
namespace civ {
namespace civloc {
void MeasurementBase::ComputeChiSquareValue() {
  //
  g_eskf->H() = H_;
  g_eskf->R() = R_;
  g_eskf->Z() = Z_;
  //
  chi_square_value_ = g_eskf->ComputeChiSquare(pre_state_);
}

void MeasurementBase::ComputeDeltaX() {
  g_eskf->H() = H_;
  g_eskf->R() = R_;
  g_eskf->Z() = Z_;
  dx_ = g_eskf->ComputeDeltaX(pre_state_);
}

void MeasurementBase::Update() {
  g_eskf->UpdateP(fused_state_);
  g_eskf->CompensateDeltaX(fused_state_, dx_);
}

spState MeasurementBase::DoMeasurement(crsp_cState pre_state,
                                       crsp_cZFrame frame,
                                       spState fused_state) {
  pre_state_ = pre_state, base_frame_ = frame, fused_state_ = fused_state;

  if (!this->ConfirmValidity()) {
    return nullptr;
  }

  this->ComputeJacobian();
  this->AdaptNoise();
  ComputeChiSquareValue();
  ComputeDeltaX();
  if (!this->DeltaXTest()) {
    return nullptr;
  }
  Update();
  return fused_state_;
}

}  // namespace civloc
}  // namespace civ
