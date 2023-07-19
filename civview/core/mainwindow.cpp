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
#include "mainwindow.h"
#include <QDebug>
#include <QTextEdit>
#include "./ui_mainwindow.h"
#include "localization_wnd.h"
#include "online_wnd.h"
#include "vtk_scene.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);

  InitUi();
  ConnectSlots();
}

void MainWindow::InitUi() {
  setWindowTitle("Localization Visualize System");
  setDockNestingEnabled(true);
  this->resize(1500, 1000);

  QSurfaceFormat::setDefaultFormat(QVTKOpenGLNativeWidget::defaultFormat());
  vtk_scene_ = new civ::civview::VTKScene(this);
  this->setCentralWidget(vtk_scene_);
  // this->ShowMapVTK("/home/baojiali/Downloads/civpilot/map_data/map.txt");
}

void MainWindow::ConnectSlots() {
  ShowOnlineSlot(true);
  ShowLocalizationSlot(true);
  // Connect the signal from localization to functions that plots on vtk
  connect(localization_wnd_, SIGNAL(trajectorySignal(QString)), this,
          SLOT(ShowTrajectoryVTK(QString)));
  connect(localization_wnd_, SIGNAL(gnssSignal(QString)), this,
          SLOT(ShowGNSSVTK(QString)));
  connect(localization_wnd_, SIGNAL(mapSignal(QString)), this,
          SLOT(ShowMapVTK(QString)));
}

void MainWindow::ShowOnlineSlot(bool checked) {
  if (!online_wnd_) {
    online_wnd_ = new OnlineWnd(this);
    online_wnd_->setWindowTitle("Online Localization");
    online_wnd_->setFeatures(
        QDockWidget::DockWidgetMovable |
        QDockWidget::DockWidgetClosable);  // 设置停靠窗口特性，可移动，可关闭
    online_wnd_->setMinimumSize(200, 100);
    addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, online_wnd_);
  }
  online_wnd_->setVisible(checked);
}

void MainWindow::ShowLocalizationSlot(bool checked) {
  if (!localization_wnd_) {
    localization_wnd_ = new LocalizationWnd(this);
    localization_wnd_->setWindowTitle("Offline Localization");
    localization_wnd_->setFeatures(
        QDockWidget::DockWidgetMovable |
        QDockWidget::DockWidgetClosable);  // 设置停靠窗口特性，可移动，可关闭
    localization_wnd_->setMinimumSize(200, 100);
    addDockWidget(Qt::DockWidgetArea::LeftDockWidgetArea, localization_wnd_);
  }
  localization_wnd_->setVisible(checked);
}

void MainWindow::ShowTrajectoryVTK(QString file_path) {
  // str.toUtf8() -> 字节数组QByteArray
  // ……data()  -> QByteArray -> char *
  // trajectory_file_path.toStdString();
  if (vtk_scene_->ShowFusedTrajectory(file_path.toStdString())) {
    localization_wnd_->SetTextEdit("Trajectory Data Loaded");
  } else {
    localization_wnd_->SetTextEdit("Trajectory Data Failed");
    // emit vktDataStatus("Trajectory Data Failed");
  }
}

void MainWindow::ShowGNSSVTK(QString file_path) {
  if (vtk_scene_->ShowGNSSTrajectory(file_path.toStdString())) {
    localization_wnd_->SetTextEdit("GNSS Data Loaded");
  } else {
    localization_wnd_->SetTextEdit("Failed Data Loaded");
  }
}

void MainWindow::ShowMapVTK(QString file_path) {
  if (vtk_scene_->ShowMap(file_path.toStdString())) {
    localization_wnd_->SetTextEdit("Map Data Loaded");
  } else {
    localization_wnd_->SetTextEdit("Map Data Failed");
  }
}

MainWindow::~MainWindow() { delete ui; }
