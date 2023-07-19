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
#include "vtk_scene.h"
#include <vtk-8.2/vtkActor.h>
#include <vtk-8.2/vtkAutoInit.h>
#include <vtk-8.2/vtkCellData.h>
#include <vtk-8.2/vtkCubeSource.h>
#include <vtk-8.2/vtkCylinderSource.h>
#include <vtk-8.2/vtkDataSetMapper.h>
#include <vtk-8.2/vtkGenericOpenGLRenderWindow.h>
#include <vtk-8.2/vtkInteractorStyleTrackballCamera.h>
#include <vtk-8.2/vtkLabeledDataMapper.h>
#include <vtk-8.2/vtkPolyDataMapper.h>
#include <vtk-8.2/vtkPolyLine.h>
#include <vtk-8.2/vtkPolyVertex.h>
#include <vtk-8.2/vtkProperty.h>
#include <vtk-8.2/vtkRenderWindow.h>
#include <vtk-8.2/vtkRenderWindowInteractor.h>
#include <vtk-8.2/vtkRenderer.h>
#include <vtk-8.2/vtkSTLReader.h>
#include <vtk-8.2/vtkSmartPointer.h>
#include <vtk-8.2/vtkSphereSource.h>
#include <vtk-8.2/vtkTextProperty.h>
#include <vtk-8.2/vtkUnsignedCharArray.h>
#include <vtk-8.2/vtkUnstructuredGrid.h>
#include <QtCore/QTimer>
#include <map>
#include <memory>
#include <vector>
#include "civmap/core/src/map.hpp"
#include "common/coordinate_transform/LocalCartesian_util.h"
#include "common/coordinate_transform/earth.hpp"
#include "common/gnss_third_party/rtklib.h"
#include "common/util/util.h"
#include "vtk_source/source_manager.hpp"
#include "vtk_source/text_source.hpp"
#include "vtk_source/vehicle_source.hpp"

namespace civ {
namespace civview {

VTKScene::VTKScene(QWidget *parent) : QVTKOpenGLNativeWidget(parent) {
  Init();
  // InitText();
}

void VTKScene::Init() {
  using namespace civ::common::coord_transform;
  Earth::SetOrigin(
      Eigen::Vector3d(g_ori_pos_deg[0], g_ori_pos_deg[1], g_ori_pos_deg[2]),
      true);
  renderer_ = vtkSmartPointer<vtkRenderer>::New();
  renderWindow_ = vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  renderWindow_->AddRenderer(renderer_);
  renderWindow_->SetWindowName("RenderWindowNoUIFile");
  renderWindow_->SetSize(1500, 1000);
  this->SetRenderWindow(renderWindow_);
  colors_ = vtkSmartPointer<vtkNamedColors>::New();
  renderer_->SetBackground(0.1, 0.2, 0.4);
  ConnectSlots();
  // show the vehicle according to the localizaiton result
  SourceManager::AddSource("vehicle", std::make_shared<VehicleSource>());
  SourceManager::AddSource("text", std::make_shared<TextSource>());
}

void VTKScene::InitText() {
  textActor_ = vtkSmartPointer<vtkTextActor>::New();
  int *winSize = renderWindow_->GetSize();
  textActor_->SetDisplayPosition(0, 0);  // Position
  textActor_->SetInput("LLH:\nENU:\nRPY");
  textActor_->GetActualPosition2Coordinate()
      ->SetCoordinateSystemToNormalizedViewport();

  textActor_->GetTextProperty()->SetFontSize(24);
  renderer_->AddActor(textActor_);
}

std::map<size_t, MsgPoint> VTKScene::ReadENUTrajectory(std::string file_path) {
  const int scale = 1;
  std::ifstream file;
  file.open(file_path.c_str(), std::ios_base::in);
  if (!file.is_open()) {
    std::cout << "fail to open trajectory file";
  }
  std::string str_data;

  std::map<size_t, MsgPoint> msgs;
  while (getline(file, str_data)) {
    std::vector<std::string> split_result =
        civ::common::util::split(str_data, ",");
    if (split_result.size() < 2) {
      continue;
    }

    double time_s = std::stod(split_result[0]);
    double time_ms = time_s * 1000;
    float East = std::stof(split_result[1]);
    float North = std::stof(split_result[2]);
    float height = std::stof(split_result[3]);

    Eigen::Vector3d pt_enu = Eigen::Vector3d{East, North, height};
    // pt_enu=G_p_Gps;
    pt_enu *= scale;
    msgs[time_ms] = MsgPoint(pt_enu[0], pt_enu[1], pt_enu[2]);
    msgs[time_ms].set_time(time_s);
  }
  return msgs;
}

bool VTKScene::ReadTrajectory(std::string file_path,
                              std::map<size_t, MsgPoint> *msgs) {
  using namespace civ::common::coord_transform;
  const int scale = 1;
  std::ifstream file;

  file.open(file_path.c_str(), std::ios_base::in);
  if (!file.is_open()) {
    std::cout << "fail to open trajectory file";
    return false;
  }
  std::string str_data;

  while (getline(file, str_data)) {
    std::vector<std::string> split_result =
        civ::common::util::split(str_data, ",");
    if (split_result.size() < 2) {
      continue;
    }

    double time_s = std::stod(split_result[0]);
    double time_ms = time_s * 1000;
    double lat_degree = std::stod(split_result[1]) * R2D;
    double lon_degree = std::stod(split_result[2]) * R2D;
    double height = std::stod(split_result[3]);
    Eigen::Vector3d pt_llh(lat_degree, lon_degree, height);

    Eigen::Vector3d G_p_Gps;

    Eigen::Vector3d pt_enu;
    ConvertLLAToENU(Earth::GetOrigin(), pt_llh, &pt_enu);

    // pt_enu=G_p_Gps;
    (*msgs)[time_ms].set_time(time_ms);
    pt_enu *= scale;
    if (mode_2d_) {
      (*msgs)[time_ms] = MsgPoint(pt_enu[0], pt_enu[1], 0);
    } else {
      (*msgs)[time_ms] = MsgPoint(pt_enu[0], pt_enu[1], pt_enu[2]);
    }

    received_enu_data_ << std::setprecision(17) << time_s << "," << pt_enu(0)
                       << "," << pt_enu(1) << "," << pt_enu(2) << std::endl;
  }
  return true;
}

void VTKScene::add_line(std::map<size_t, MsgPoint> msgs,
                        vtkSmartPointer<vtkActor> &actor_,
                        std::string color = "White") {
  vtkSmartPointer<vtkUnstructuredGrid> ploy_data =
      vtkSmartPointer<vtkUnstructuredGrid>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  vtkSmartPointer<vtkPolyLine> line = vtkSmartPointer<vtkPolyLine>::New();
  vtkSmartPointer<vtkPolyVertex> vtx = vtkSmartPointer<vtkPolyVertex>::New();

  points->SetNumberOfPoints(msgs.size());
  line->GetPointIds()->SetNumberOfIds(msgs.size());
  vtx->GetPointIds()->SetNumberOfIds(msgs.size());
  int count = 0;
  for (auto &[time, trajectory] : msgs) {
    points->SetPoint(count, trajectory.x(), trajectory.y(), trajectory.z());
    line->GetPointIds()->SetId(count, count);
    vtx->GetPointIds()->SetId(count, count);
    count++;
  }

  ploy_data->SetPoints(points);
  ploy_data->InsertNextCell(line->GetCellType(), line->GetPointIds());
  ploy_data->InsertNextCell(vtx->GetCellType(), vtx->GetPointIds());
  vtkSmartPointer<vtkNamedColors> colors_ =
      vtkSmartPointer<vtkNamedColors>::New();
  vtkSmartPointer<vtkUnsignedCharArray> cellColors =
      vtkSmartPointer<vtkUnsignedCharArray>::New();
  cellColors->SetNumberOfComponents(3);
  cellColors->InsertNextTypedTuple(colors_->GetColor3ub(color).GetData());
  cellColors->InsertNextTypedTuple(colors_->GetColor3ub(color).GetData());
  ploy_data->GetCellData()->SetScalars(cellColors);
  vtkSmartPointer<vtkDataSetMapper> mapper_ =
      vtkSmartPointer<vtkDataSetMapper>::New();
  // mapper_->SetInputData(ploy_data);
  mapper_->AddInputDataObject(ploy_data);

  actor_->SetMapper(mapper_);
  actor_->GetProperty()->SetPointSize(5);
}

void VTKScene::add_map(std::vector<sp_cZMapLineSegment> lines) {
  int count = 0;
  const int scale = 1;
  for (const auto &line : lines) {
    std::map<size_t, MsgPoint> msgs;
    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    for (const auto &pt : line->points_) {
      // scale to reduce
      if (mode_2d_) {
        msgs[count] = MsgPoint(pt[0] * scale, pt[1] * scale, 0 * scale);
      } else {
        msgs[count] = MsgPoint(pt[0] * scale, pt[1] * scale, pt[2] * scale);
      }
      count++;
    }
    add_line(msgs, actor, "White");
    renderer_->AddActor(actor);
  }
}

bool VTKScene::ShowMap(std::string file_path) {
  using civ::civmap::CivMap;
  using civ::civmap::spCivMap;
  spCivMap sp_map = std::make_shared<CivMap>();
  if (!sp_map->ReadData(file_path)) {
    return false;
  }
  using civ::civmap::sp_cZMapLineSegment;
  std::vector<sp_cZMapLineSegment> lines = sp_map->get_lines_enu();
  this->add_map(lines);
  renderer_->ResetCamera();
  this->GetRenderWindow()->Render();
  return true;
}

bool VTKScene::ShowGNSSTrajectory(std::string file_path) {
  std::cout << "show gnss trajectory" << std::endl;
  vtkSmartPointer<vtkActor> actor_gnss = vtkSmartPointer<vtkActor>::New();
  std::map<size_t, MsgPoint> msgs_gnss;
  if (!ReadTrajectory(file_path, &msgs_gnss)) {
    return false;
  }
  this->add_line(msgs_gnss, actor_gnss, "Yellow");
  renderer_->AddActor(actor_gnss);
  renderer_->ResetCamera();
  this->GetRenderWindow()->Render();
  return true;
}
bool VTKScene::ShowFusedTrajectory(std::string file_path) {
  std::cout << "show fused trajectory" << std::endl;
  vtkSmartPointer<vtkActor> actor_traj = vtkSmartPointer<vtkActor>::New();
  std::map<size_t, MsgPoint> msgs_traj;
  if (!ReadTrajectory(file_path, &msgs_traj)) {
    return false;
  }
  this->add_line(msgs_traj, actor_traj, "Red");
  renderer_->AddActor(actor_traj);
  renderer_->ResetCamera();
  this->GetRenderWindow()->Render();
  return true;
}

void VTKScene::ConnectSlots() {
  // update the contents in VTK every interval
  timer_ = new QTimer(this);
  timer_->setInterval(100 * 1);  // refreshing interval, unit ms
  timer_->start();
  connect(timer_, &QTimer::timeout, this, &VTKScene::Render);
}

void VTKScene::Render() {
  ccc+=1;
  std::cout<<"rendering:"<<ccc<<std::endl;
  SourceManager::RefreshAllActor(renderer_);
  this->GetRenderWindow()->Render();
}

}  // namespace civview
}  // namespace civ
