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
#include <vtk-8.2/vtkRenderer.h>
#include <vtk-8.2/vtkSmartPointer.h>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include "vtk_source/source_base.hpp"
#include "vtk_source/trajectory_source.hpp"
namespace civ {
namespace civview {
class SourceManager {
 public:
  static SourceManager* Instance();
  static std::shared_ptr<SourceBase> AddSource(
      std::string const& channel_name, std::shared_ptr<SourceBase> source);
  static void RefreshAllActor(vtkSmartPointer<vtkRenderer>& render);
  static void set_main_trj(std::shared_ptr<TrajectorySource> source);

 private:
  static SourceManager* instance_;

 private:
  std::map<std::string, std::shared_ptr<SourceBase>> all_source_;
  std::mutex mtx_all_source_;
  std::shared_ptr<TrajectorySource> main_trj_ = nullptr;
  std::shared_ptr<Eigen::Isometry3d> vehicle_pose_ = nullptr;
  std::map<std::string, std::shared_ptr<Eigen::Isometry3d>> trans_external_;
  std::string external_path_ = "";
};
}  // namespace civview
}  // namespace civ