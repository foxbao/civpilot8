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
#include "vtk_source/trajectory_source.hpp"
#include <vtkPolyLine.h>
#include <vtkPolyVertex.h>
#include <vtkUnsignedCharArray.h>
#include <iostream>
#include <map>
#include <memory>
#include <string>

namespace civ {
namespace civview {

void TrajectorySource::GenerateActorImpl(std::map<size_t, sp_cMsg2>& msgs) {
  std::string color;
  switch (lane_color_) {
    case 0:
      color = "Yellow";
      break;
    case 1:
      color = "Red";
      break;
    case 2:
      color = "Blue";
      break;
    case 3:
      color = "Green";
      break;
    default:
      color = "White";
      break;
  }
  ploy_data_ = vtkSmartPointer<vtkUnstructuredGrid>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkPolyLine> line = vtkSmartPointer<vtkPolyLine>::New();
  vtkSmartPointer<vtkPolyVertex> vtx = vtkSmartPointer<vtkPolyVertex>::New();
  points->SetNumberOfPoints(msgs.size());
  line->GetPointIds()->SetNumberOfIds(msgs.size());
  vtx->GetPointIds()->SetNumberOfIds(msgs.size());
  int count = 0;

  std::shared_ptr<Eigen::Isometry3d> lastest_pose = get_lastest_pose();

  for (auto& [time, trajectory] : msgs) {
    auto ploc = std::dynamic_pointer_cast<ProtoType const>(trajectory->msg_);
    // double mea_time=ploc->measurement_time();
    auto pt = ploc->pose().position();
    auto ori = ploc->pose().orientation();

    if (mode_2d_) {
      points->SetPoint(count, pt.x(), pt.y(), 0);
      // vehicle_->SetPose(pt.x(), pt.y(), 0);
    } else {
      points->SetPoint(count, pt.x(), pt.y(), pt.z());
      // vehicle_->SetPose(pt.x(), pt.y(), pt.z());
    }

    // std::cout<<mea_time<<pt.x()<<","<<pt.y()<<","<<pt.z()<<std::endl;
    line->GetPointIds()->SetId(count, count);
    vtx->GetPointIds()->SetId(count, count);
    count++;
  }
  ploy_data_->SetPoints(points);
  ploy_data_->InsertNextCell(line->GetCellType(), line->GetPointIds());
  ploy_data_->InsertNextCell(vtx->GetCellType(), vtx->GetPointIds());
  vtkSmartPointer<vtkUnsignedCharArray> cellColors =
      vtkSmartPointer<vtkUnsignedCharArray>::New();
  cellColors->SetNumberOfComponents(3);
  cellColors->InsertNextTypedTuple(colors_->GetColor3ub("Red").GetData());
  cellColors->InsertNextTypedTuple(colors_->GetColor3ub(color).GetData());
  ploy_data_->GetCellData()->SetScalars(cellColors);
  // ------------
  mapper_ = vtkSmartPointer<vtkDataSetMapper>::New();
  mapper_->SetInputData(ploy_data_);
  // mapper_->AddInputDataObject();

  actor_ = vtkSmartPointer<vtkActor>::New();
  actor_->SetMapper(mapper_);
  actor_->GetProperty()->SetPointSize(5);
}

sp_cMsg2 TrajectorySource::AdapteProto(sp_cMsg msg) {
  
  auto concrete_msg = std::dynamic_pointer_cast<ProtoType const>(msg);
  Msg2 msg2;
  msg2.t0_ = concrete_msg->measurement_time();
  msg2.t1_ = concrete_msg->header().timestamp_sec();
  msg2.msg_ = msg;
  return std::make_shared<Msg2>(msg2);
}

std::shared_ptr<Eigen::Isometry3d> TrajectorySource::get_lastest_pose() {
  auto lastest_elem = this->get_lastest_elem();
  if (!lastest_elem) {
    return nullptr;
  }
  auto lastest_pose =
      std::dynamic_pointer_cast<ProtoType const>(lastest_elem->msg_);
  auto pose =
      std::make_shared<Eigen::Isometry3d>(Eigen::Isometry3d::Identity());
  auto const& pose_proto = lastest_pose->pose();
  auto const& pos_proto = lastest_pose->pose().position();
  auto const& qua_proto = lastest_pose->pose().orientation();
  Eigen::Quaterniond qua = Eigen::Quaterniond{qua_proto.qw(), qua_proto.qx(),
                                              qua_proto.qy(), qua_proto.qz()};

  pose->translation() =
      Eigen::Vector3d{pos_proto.x(), pos_proto.y(), pos_proto.z()};
  pose->linear() = qua.toRotationMatrix();
  return pose;
}

}  // namespace civview
}  // namespace civ
