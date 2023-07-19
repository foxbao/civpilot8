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
#pragma once
#include <vtkActor.h>
#include <vtkCellData.h>
#include <vtkDataSetMapper.h>
#include <vtkDoubleArray.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkProperty.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtkUnstructuredGrid.h>
#include <atomic>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <future>
#include <memory>
#include <string>
#include <vector>

#include <QtCore/QObject>
#include "cyber/common/log.h"
#include "vtk_source/proto_type_adapter.hpp"
namespace civ {
namespace civview {
enum class DataStatus {
  NONE = 0,
  LOADING,   // 正在载入
  QUERYING,  // 正在查找
  UPDATING,  // 正在更新
  READY
};
class IFileLoader : public QObject {
  Q_OBJECT
 public:
  virtual void LoadFromFile(std::string const& path) {
    AINFO << "virtual void LoadFromFile " << path;
  }
  void LoadFromFiles(std::vector<std::string> const& paths);
  virtual void LoadFromEpoch(double epoch) {}
  DataStatus get_load_status() const { return data_status_; }
  bool isDataReady() const { return data_status_ == DataStatus::READY; }

 protected:
  std::atomic<DataStatus> data_status_ = {DataStatus::NONE};
  std::future<void> future_load_from_file_;

 public:
 Q_SIGNALS:
  void DataLoaded(std::string str);  // 数据加载完毕后，出发窗口更新数据使用
                                     // [例如轨迹加载完成，更新起止时间]
 protected:
  // virtual void OnDataLoaded() = 0;  // 数据加载完毕，更新actor
};

enum class SourceBaseMode {
  NONE = 0,
  ONLINE = 5,     // 在线刷新
  FILE_LOAD = 10  // 文件读取
};

class SourceBase : public IFileLoader {
 public:
  SourceBase(SourceBaseMode const& mode, std::string const& path)
      : mode_(mode), path_(path) {}
  void AppendData(sp_cMsg msg);
  void ClearData();
  void set_max_size(size_t n) { max_size_ = n; }
  void set_mode(SourceBaseMode mode) {
    mode_ = mode;
  }  // 确认是文件读取或是实时读取
  void RefreshActorInRender(vtkSmartPointer<vtkRenderer>& render);
  void UpdateVehiclePose(std::shared_ptr<Eigen::Isometry3d> pose);
  void UpdateTransExternal(std::shared_ptr<Eigen::Isometry3d> pose);

 protected:
  virtual void RefreshActorByOnlineImpl(vtkSmartPointer<vtkRenderer>& render);

  //   void RefreshActorByLoadFileImpl(vtkSmartPointer<vtkRenderer>& render);

 protected:
  bool GenerateActor();
  virtual sp_cMsg2 AdapteProto(sp_cMsg msg) = 0;
  virtual void GenerateActorImpl(std::map<size_t, sp_cMsg2>& msgs) = 0;
  sp_cMsg2 get_lastest_elem();
    void UpdateSourceInWorld(vtkSmartPointer<vtkActor>& actor);


 protected:
  SourceBase() {}
  ~SourceBase() {}
  std::map<size_t, sp_cMsg2> map_data_;  // 所有按时间顺序proto数据
  size_t max_size_ = 1;
  vtkSmartPointer<vtkNamedColors> colors_ =
      vtkSmartPointer<vtkNamedColors>::New();
  vtkSmartPointer<vtkUnstructuredGrid> ploy_data_ = nullptr;
  vtkSmartPointer<vtkDataSetMapper> mapper_ = nullptr;
  vtkSmartPointer<vtkActor> actor_ = nullptr;
  vtkSmartPointer<vtkActor> actor_tobe_remove_ = nullptr;  // 即将被删除的actor
  //   std::atomic<bool> is_actor_added_ = {false};  //
  //   当前actor是否已经加入render
  std::shared_ptr<Eigen::Isometry3d> vehicle_pose_ = nullptr;  // 车体位姿
  std::shared_ptr<Eigen::Isometry3d> trans_obj_to_vehicle_ =
      nullptr;  // obj在车体坐标系下的位姿
  SourceBaseMode mode_ = SourceBaseMode::ONLINE;
  std::string path_;  // 通道名或者path
  int lane_color_ = 3;
  bool mode_2d_ = true;
  std::mutex mtx_map_data_;
 private:
  
  
};
}  // namespace civview
}  // namespace civ
