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
#include "vtk_source/vehicle_source.hpp"
#include <vtk-8.2/vtkCubeSource.h>
#include <vtk-8.2/vtkPolyDataMapper.h>
#include <vtk-8.2/vtkSTLReader.h>
#include <vtk-8.2/vtkSmartPointer.h>
#include <memory>
namespace civ {
namespace civview {
VehicleSource::VehicleSource() {
  data_status_ = DataStatus::READY;
  map_data_[0] = std::make_shared<Msg2>();

  std::string file_path =
      "/home/baojiali/Downloads/civpilot/civview/model/ShelbyWD.stl";

  vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
  reader->SetFileName(file_path.c_str());
  reader->Update();
  // vtkSmartPointer<vtkPolyDataMapper> mapper =
  //     vtkSmartPointer<vtkPolyDataMapper>::New();
  // mapper->SetInputConnection(reader->GetOutputPort());

  mapper_ = vtkSmartPointer<vtkDataSetMapper>::New();
  mapper_->SetInputConnection(reader->GetOutputPort());
  actor_ = vtkSmartPointer<vtkActor>::New();
  actor_->SetMapper(mapper_);
  // vehicle_actor_ = vtkSmartPointer<vtkActor>::New();
  // vehicle_actor_->SetMapper(mapper);
  // actor_->GetProperty()->SetColor(0.272, 0.456, 0.272);
  actor_->SetOrientation(0, 0, -90);

}

void VehicleSource::GenerateActorImpl(std::map<size_t, sp_cMsg2>& msgs) {
  // 转换至世界坐标
  this->UpdateSourceInWorld(actor_);
}


bool VehicleSource::ReadFile(std::string file_path) {
  file_path_ = file_path;
  vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
  reader->SetFileName(file_path.c_str());
  reader->Update();
  vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(reader->GetOutputPort());

  actor_ = vtkSmartPointer<vtkActor>::New();
  actor_->SetMapper(mapper);
  actor_->GetProperty()->SetColor(0.272, 0.456, 0.272);
  return true;
}


sp_cMsg2 VehicleSource::AdapteProto(sp_cMsg msg) {
  Msg2 msg2;
  return std::make_shared<Msg2>(msg2);
}

}  // namespace civview
}  // namespace civ
