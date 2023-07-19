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

#include <QLine>
#include <QLineEdit>
#include <QPaintEvent>
#include <QPen>
#include <QPushButton>
#include <QTextEdit>
#include <QtWidgets/QDockWidget>
class LocalizationWnd : public QDockWidget {
  Q_OBJECT
 public:
  LocalizationWnd(QWidget* parent = nullptr);
  void SetTextEdit(QString context);

 protected:
  // void paintEvent(QPaintEvent *event)
  // {
  //     QPainter painter(this);
  //     painter.setPen(QPen(Qt::black, 12, Qt::DashDotLine, Qt::RoundCap));
  //     painter.drawLine(0, 0, 200, 200);
  // }
 private:
  QLineEdit* ptr_line_trajectory_file_path_ = nullptr;
  QPushButton* ptr_btn_load_trajectory_ = nullptr;
  QPushButton* ptr_btn_clear_trajectory_ = nullptr;
  QLineEdit* ptr_line_gnss_file_path_ = nullptr;
  QPushButton* ptr_btn_load_gnss_ = nullptr;
  QLineEdit* ptr_line_map_file_path_ = nullptr;
  QPushButton* ptr_btn_load_map_ = nullptr;
  QTextEdit* ptr_text_edit_status_ = nullptr;

 Q_SIGNALS:
  /* 信号必须有signals关键字来声明
   * 信号没有返回值，但可以有参数
   * 信号就是函数的声明，只需声明，无需定义
   * 使用：emit trajectorySignal();
   * 信号可以重载
   */
  // void trajectorySignal();
  void trajectorySignal(QString);
  void gnssSignal(QString);
  void mapSignal(QString);
  // void trajectorySignal(int, QString);

 private:
  void InitUi();
  void ConectSlots();

 private Q_SLOTS:
  void LoadButtonTrajectoryClicked();
  void LoadButtonGNSSClicked();
  void LoadButtonMapClicked();
};
