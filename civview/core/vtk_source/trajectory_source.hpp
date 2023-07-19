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
#include <map>
#include <memory>
#include <string>
#include "message/localization/proto/localization.pb.h"
#include "vtk_source/source_base.hpp"
// #include "vtk_source/vehicle_source.hpp"
#pragma once
namespace civ {
namespace civview {
class TrajectorySource : public SourceBase {
 public:
  using SourceBase::SourceBase;
  using ProtoType = civ::localization::LocalizationEstimate;
  TrajectorySource(SourceBaseMode const& mode, std::string const& path)
      : SourceBase(mode, path) {
    // std::string vehicle_model_file_path =
        // "/home/baojiali/Downloads/civpilot/civview/model/ShelbyWD.stl";
    // vehicle_ = std::make_shared<VehicleSource>(vehicle_model_file_path);
  }

 public:
  std::shared_ptr<Eigen::Isometry3d> get_lastest_pose();

 protected:
  void GenerateActorImpl(std::map<size_t, sp_cMsg2>& msgs) override;
  sp_cMsg2 AdapteProto(sp_cMsg msg) override;
  

 private:
  // std::shared_ptr<VehicleSource> vehicle_;
};
}  // namespace civview
}  // namespace civ
