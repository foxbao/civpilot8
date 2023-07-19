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
#include <vtk-8.2/QVTKOpenGLNativeWidget.h>
#include <vtk-8.2/vtkNamedColors.h>
#include <vtk-8.2/vtkRenderer.h>
#include <vtk-8.2/vtkSmartPointer.h>
#include <vtk-8.2/vtkTextActor.h>
#include <map>
#include <string>
#include <vector>
#include "common/coordinate_transform/earth.hpp"
#include "civmap/core/src/inner_types.hpp"
#include "civmap/core/src/map.hpp"
namespace civ {
namespace civview {
using civ::civmap::sp_cZMapLineSegment;
class MsgPoint {
 public:
  MsgPoint() {}
  MsgPoint(double x, double y, double z) : x_(x), y_(y), z_(z) {}

 public:
  int id() { return id_; }
  double x() { return x_; }
  double y() { return y_; }
  double z() { return z_; }
  double time() { return time_; }
  void set_time(double time) { time_ = time; }

 private:
  int id_;
  double time_;
  double x_;
  double y_;
  double z_;
};

class VTKScene : public QVTKOpenGLNativeWidget {
  Q_OBJECT
 public:
  explicit VTKScene(QWidget* parent = nullptr);
  vtkSmartPointer<vtkRenderer>& get_render() { return renderer_; }
  void ResetGridSizeOn() { grid_size_on_ = true; }
  bool ShowFusedTrajectory(
      std::string file_path);  // activated by mainwindow through clicing button
  bool ShowGNSSTrajectory(
      std::string file_path);  // activated by mainwindow through clicing button
  bool ShowMap(
      std::string file_path);  // activated by mainwindow through clicing button
  // bool ShowVehicle(std::string file_path);
  void Render();
  

 private:
  bool ReadTrajectory(std::string file_path, std::map<size_t, MsgPoint>* msgs);
  std::map<size_t, MsgPoint> ReadENUTrajectory(std::string file_path);
  void add_map(std::vector<sp_cZMapLineSegment> lines);
  void add_line(std::map<size_t, MsgPoint> msgs,
                vtkSmartPointer<vtkActor>& actor_, std::string color);
  void ConnectSlots();
  void Init();
  void InitText();
  QTimer* timer_;

 private:
  vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow_;
  vtkSmartPointer<vtkRenderer> renderer_;
  vtkSmartPointer<vtkNamedColors> colors_;
  vtkSmartPointer<vtkTextActor> textActor_;

  std::atomic_bool grid_size_on_ = {false};
  bool mode_2d_ = true;
  std::string received_enu_data_path_;
  std::ofstream received_enu_data_;
  double ccc=0;
};
}  // namespace civview
}  // namespace civ
