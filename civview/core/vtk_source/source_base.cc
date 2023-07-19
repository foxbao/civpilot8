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

#include "vtk_source/source_base.hpp"
#include <map>
namespace civ {
namespace civview {
void IFileLoader::LoadFromFiles(std::vector<std::string> const& paths) {
  // 多线程一起读，线程同步
}
void SourceBase::UpdateVehiclePose(std::shared_ptr<Eigen::Isometry3d> pose) {
  //
  vehicle_pose_ = pose;
}
void SourceBase::UpdateTransExternal(
    std::shared_ptr<Eigen::Isometry3d> trans_external) {
  //
  trans_obj_to_vehicle_ = trans_external;
}

void SourceBase::UpdateSourceInWorld(vtkSmartPointer<vtkActor>& actor) {
  if (vehicle_pose_) {
    // transform
    Eigen::Isometry3d T_world_object = *vehicle_pose_;  // T_w_v
    if (trans_obj_to_vehicle_) {
      T_world_object =
          T_world_object * (*trans_obj_to_vehicle_);  // T_w_o =  T_w_v * T_v_o
    }
    // // 内存顺序修改为行主序，data()的内存顺序需要与vtkTransform保持一致
    Eigen::Matrix<double, 4, 4, Eigen::RowMajor> row_matrix =
        T_world_object.matrix();

    if (mode_2d_) {
      row_matrix(2, 3) = 0;
    }
    vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
    trans->SetMatrix(row_matrix.data());

    if (actor_) {
      actor_->SetUserTransform(trans);
    } else {
      printf("no actor_\n");
    }
  }
}

sp_cMsg2 SourceBase::get_lastest_elem() {
  std::unique_lock<std::mutex> lock(mtx_map_data_);
  return map_data_.empty() ? nullptr : map_data_.rbegin()->second;
}

void SourceBase::AppendData(sp_cMsg msg) {
  // adapter
  auto msg2 = this->AdapteProto(msg);
  msg2->t0_;
  {
    std::unique_lock<std::mutex> lock(mtx_map_data_);
    map_data_[msg2->get_t0_ms()] = msg2;
    // size check
    if (map_data_.size() > max_size_) {
      map_data_.erase(map_data_.begin());
    }
  }
}

void SourceBase::ClearData() {
  std::unique_lock<std::mutex> lock(mtx_map_data_);
  map_data_.clear();
}

bool SourceBase::GenerateActor() {
  std::map<size_t, sp_cMsg2> msgs_copy;
  {
    std::unique_lock<std::mutex> lock(mtx_map_data_);
    msgs_copy = map_data_;
  }
  if (!msgs_copy.empty()) {
    if (actor_) {
      actor_tobe_remove_ = actor_;
    }
    this->GenerateActorImpl(msgs_copy);
  }
  return actor_;
}

void SourceBase::RefreshActorByOnlineImpl(
    vtkSmartPointer<vtkRenderer>& render) {
  if (GenerateActor()) {
    if (actor_tobe_remove_) {
      render->RemoveActor(actor_tobe_remove_);
      actor_tobe_remove_ = nullptr;
    }
    render->AddActor(actor_);
  }
}
// // 需要改成时间触发
// void SourceBase::RefreshActorByLoadFileImpl(
//     vtkSmartPointer<vtkRenderer>& render) {
//   if (data_status_ != DataStatus::READY) {
//     return;
//   }

//   // 当前actor未加入
//   if (!is_actor_added_) {
//     GenerateActor();

//     render->RemoveActor(actor_tobe_remove_);
//     actor_tobe_remove_ = nullptr;
//     render->AddActor(actor_);

//     is_actor_added_ = true;
//   }
// }

void SourceBase::RefreshActorInRender(vtkSmartPointer<vtkRenderer>& render) {
  if (mode_ == SourceBaseMode::ONLINE) {
    RefreshActorByOnlineImpl(render);
  } else if (mode_ == SourceBaseMode::FILE_LOAD) {
    // RefreshActorByLoadFileImpl(render);
  } else {
    AERROR << "bad RefreshActorInRender mode";
  }
}
}  // namespace civview
}  // namespace civ
