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
#include "localization_wnd.h"
#include <QGridLayout>
#include <QLabel>
#include <QPicture>
#include <iostream>
#include <string>
LocalizationWnd::LocalizationWnd(QWidget* parent) : QDockWidget(parent) {
  InitUi();
  ConectSlots();
}

void LocalizationWnd::InitUi() {
  ptr_line_trajectory_file_path_ = new QLineEdit(this);
  ptr_line_trajectory_file_path_->setText(
      "/home/baojiali/Downloads/civpilot/data/fused_result.txt");
  ptr_btn_load_trajectory_ = new QPushButton(tr("Load Trajectory"), this);
  // ptr_btn_clear_trajectory_ = new QPushButton(tr("Clear Trajecotry"), this);

  ptr_line_gnss_file_path_ = new QLineEdit(this);
  ptr_line_gnss_file_path_->setText(
      "/home/baojiali/Downloads/civpilot/data/gnss_result.txt");
  ptr_btn_load_gnss_ = new QPushButton(tr("Load Gnss"), this);
  ptr_line_map_file_path_ = new QLineEdit(this);
  ptr_line_map_file_path_->setText(
      "/home/baojiali/Downloads/civpilot/map_data/map.txt");
  ptr_btn_load_map_ = new QPushButton(tr("Load Map"), this);
  ptr_text_edit_status_ = new QTextEdit(tr(""), this);

  QWidget* p_main_wiget = new QWidget(this);
  QGridLayout* p_main_grid = new QGridLayout(p_main_wiget);
  p_main_grid->addWidget(ptr_line_trajectory_file_path_, 1, 0);
  p_main_grid->addWidget(ptr_btn_load_trajectory_, 2, 0);
  p_main_grid->addWidget(ptr_line_gnss_file_path_, 3, 0);
  p_main_grid->addWidget(ptr_btn_load_gnss_, 4, 0);
  p_main_grid->addWidget(ptr_line_map_file_path_, 5, 0);
  p_main_grid->addWidget(ptr_btn_load_map_, 6, 0);
  p_main_grid->addWidget(ptr_text_edit_status_, 7, 0);
  p_main_grid->setSpacing(10);
  this->setWidget(p_main_wiget);
}

void LocalizationWnd::ConectSlots() {
  connect(ptr_btn_load_trajectory_, &QPushButton::clicked, this,
          &LocalizationWnd::LoadButtonTrajectoryClicked);
  connect(ptr_btn_load_gnss_, &QPushButton::clicked, this,
          &LocalizationWnd::LoadButtonGNSSClicked);
  connect(ptr_btn_load_map_, &QPushButton::clicked, this,
          &LocalizationWnd::LoadButtonMapClicked);
}

void LocalizationWnd::LoadButtonTrajectoryClicked() {
  // send the signal back to mainWindow, and the mainWindow will send the signal
  // to vtk
  std::string trajectory_file_path =
      ptr_line_trajectory_file_path_->text().toStdString();
  emit trajectorySignal(QString::fromStdString(trajectory_file_path));
}

void LocalizationWnd::LoadButtonGNSSClicked() {
  std::string gnss_file_path = ptr_line_gnss_file_path_->text().toStdString();
  emit gnssSignal(QString::fromStdString(gnss_file_path));
}

void LocalizationWnd::LoadButtonMapClicked() {
  std::string map_file_path = ptr_line_map_file_path_->text().toStdString();
  emit mapSignal(QString::fromStdString(map_file_path));
}

void LocalizationWnd::SetTextEdit(QString context) {
  if (ptr_text_edit_status_) {
    ptr_text_edit_status_->setText(context);
  }
}
