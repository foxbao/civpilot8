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
#include <QApplication>
#include <vtk-8.2/vtkAutoInit.h>
#include "cyber/cyber.h"
#include "civview/core/mainwindow.h"

// float convert_hhmmmm_hh(float hhmmmm) {
//   int hh = static_cast<int>(hhmmmm / 100);
//   float mmmm = hhmmmm - hh * 100;
//   float degree = hh + mmmm / 60;
//   return degree;
// }

VTK_MODULE_INIT(vtkRenderingOpenGL2);  // VTK was built with vtkRenderingOpenGL2
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2);

int main(int argc, char *argv[]) {
  apollo::cyber::Init("control_board");
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
