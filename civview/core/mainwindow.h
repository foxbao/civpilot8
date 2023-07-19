#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtWidgets/QDockWidget>
#include "vtk_scene.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class LocalizationWnd;
class OnlineWnd;
// VTK_MODULE_INIT(vtkRenderingOpenGL2);  // VTK was built with vtkRenderingOpenGL2
// VTK_MODULE_INIT(vtkInteractionStyle);
class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  void InitUi();
  void ConnectSlots();

 private:
  int cccc = 32;

   Q_SIGNALS:
    void vktDataStatus(QString);
 private Q_SLOTS:
  void ShowLocalizationSlot(bool checked);
  void ShowOnlineSlot(bool checked);
  void ShowTrajectoryVTK(QString file_path);
  void ShowGNSSVTK(QString file_path);
  void ShowMapVTK(QString file_path);
  

 private:
  Ui::MainWindow *ui;
  LocalizationWnd *localization_wnd_{nullptr};
  OnlineWnd *online_wnd_{nullptr};
  civ::civview::VTKScene *vtk_scene_ = nullptr;

};
#endif  // MAINWINDOW_H

