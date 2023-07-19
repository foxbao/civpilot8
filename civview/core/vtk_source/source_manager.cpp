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
#include "vtk_source/source_manager.hpp"
namespace civ {
namespace civview {
SourceManager* SourceManager::instance_ = nullptr;

SourceManager* SourceManager::Instance() {
  if (!instance_) {
    instance_ = new SourceManager();
  }
  return instance_;
}
std::shared_ptr<SourceBase> SourceManager::AddSource(
    std::string const& channel_name, std::shared_ptr<SourceBase> source) {
  std::unique_lock<std::mutex> lock(Instance()->mtx_all_source_);
  Instance()->all_source_[channel_name] = source;
  return source;
}
void SourceManager::set_main_trj(std::shared_ptr<TrajectorySource> source) {
  Instance()->main_trj_ = source;
}

void SourceManager::RefreshAllActor(vtkSmartPointer<vtkRenderer>& render) {
  std::map<std::string, std::shared_ptr<SourceBase>> all_source_back;
  {
    std::unique_lock<std::mutex> lock(Instance()->mtx_all_source_);
    all_source_back = Instance()->all_source_;
  }
  if (Instance()->main_trj_) {
    Instance()->vehicle_pose_ = Instance()->main_trj_->get_lastest_pose();
  }

  int roadoutColor = 0;
  for (auto& [name, source] : all_source_back) {
    source->UpdateVehiclePose(Instance()->vehicle_pose_);
    // source->UpdateTransExternal(Instance()->trans_external_[name]);
    // // 2. 确认传感器在车上的外参
    // // 3. refresh actor
    // source->SetRoadOutColor(roadoutColor);
    source->RefreshActorInRender(render);
    
    // roadoutColor++;
  }
}
}  // namespace civview
}  // namespace civ
