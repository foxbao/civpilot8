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

#include "vtk_source/text_source.hpp"
#include <vtk-8.2/vtkTextProperty.h>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include "common/gnss_third_party/rtklib.h"
namespace civ {
namespace civview {
TextSource::TextSource() {
  data_status_ = DataStatus::READY;
  map_data_[0] = std::make_shared<Msg2>();
  textActor_ = vtkSmartPointer<vtkTextActor>::New();
  //   int *winSize = renderWindow_->GetSize();
  textActor_->SetDisplayPosition(0, 0);  // Position
  textActor_->SetInput("LLH:\nENU:\nRPY");
  textActor_->GetActualPosition2Coordinate()
      ->SetCoordinateSystemToNormalizedViewport();
  textActor_->GetTextProperty()->SetFontSize(24);
}
bool TextSource::GenerateActor() {
  std::map<size_t, sp_cMsg2> msgs_copy;
  {
    std::unique_lock<std::mutex> lock(mtx_map_data_);
    msgs_copy = map_data_;
  }
  if (!msgs_copy.empty()) {
    if (textActor_) {
      textActor_tobe_remove_ = textActor_;
    }
    this->GenerateActorImpl(msgs_copy);
  }
  return textActor_;
}

void TextSource::RefreshActorByOnlineImpl(
    vtkSmartPointer<vtkRenderer>& render) {
  // if (GenerateActor()) {
  //   if (textActor_tobe_remove_) {
  //     render->RemoveActor(textActor_tobe_remove_);
  //     textActor_tobe_remove_ = nullptr;
  //   }
  //   render->AddActor(textActor_);
  // }
}
void TextSource::GenerateActorImpl(std::map<size_t, sp_cMsg2>& msgs) {
  // calculate llh, enu and RPY from vehicle pose
  if (vehicle_pose_) {
    Eigen::Vector3d enu = vehicle_pose_->translation();
    Eigen::Matrix3d rotation = vehicle_pose_->rotation();

    // 旋转矩阵 --> 欧拉角(Z-Y-X，即RPY)（确保pitch的范围[-pi/2, pi/2]）
    Eigen::Vector3d eulerAngle_mine;
    Eigen::Matrix3d rot = rotation;
    eulerAngle_mine(2) = std::atan2(rot(2, 1), rot(2, 2));
    eulerAngle_mine(1) = std::atan2(
        -rot(2, 0), std::sqrt(rot(2, 1) * rot(2, 1) + rot(2, 2) * rot(2, 2)));
    eulerAngle_mine(0) = std::atan2(rot(1, 0), rot(0, 0));

    Eigen::Matrix<double, 3, 1> rpy = rotation.eulerAngles(0, 1, 2);
    std::stringstream ss_enu;
    ss_enu << enu.transpose();
    std::string str_enu = "ENU:" + ss_enu.str();
    std::stringstream ss_rpy;
    ss_rpy << rpy.transpose() * R2D;
    std::string str_RPY = "RPY:" + ss_rpy.str();
    std::string str_show = str_enu + "\n" + str_RPY;
    // str_enu<<enu(0);
    textActor_->SetInput(str_show.c_str());
  }
}
sp_cMsg2 TextSource::AdapteProto(sp_cMsg msg) {}
}  // namespace civview
}  // namespace civ
