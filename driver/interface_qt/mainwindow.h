/* Copyright */

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QButtonGroup>
#include <QLabel>
#include <QLineEdit>
#include <QMainWindow>
#include <QPushButton>
#include <QRadioButton>
#include <QTextEdit>
#include <future>
#include <memory>
#include "common/recorder/recorder_component.h"
#include "gnss/R9300/gnss_component.h"
#include "imu/wheeltecN100/wheeltec_component.h"
#include "qianxun/qianxun_component.h"
using civ::drivers::gnss::GnssComponent;
using civ::drivers::gnss::QianxunComponent;
using civ::drivers::wheeltec::WheeltecComponent;
using civ::drivers::common::RecorderComponent;
QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

 private:
  void InitReader();
  void InitRecorder();
  void InitUi();
  void InitUIPos();
  void InitUiGNSS();
  void InitUiIMU();
  void InitUiRecorder();
  void InitSpeedoIMU();
  void GetContentSlot();
  void executeLinuxCmd(QString strCmd, QString &strResult,
                       QString &strErrorInfo);

 private Q_SLOTS:
  void StartGnss();
  void StopGnss();
  void ShowGnssData(QString gnss_data);
  void StartIMU();
  void StopIMU();
  void ShowIMUData(QString imu_data);
  void StartRecorder();
  void StopRecorder();
  void ShowRecordingData(QString record_data);

 private:
  Ui::MainWindow *ui;
  QLabel *p_label_gnss_dev_;
  QLabel *p_label_gnss_baud_rate_;
  QLineEdit *p_line_edit_gnss_dev_;
  QLineEdit *p_line_edit_gnss_baud_rate_;
  QPushButton *p_but_gnss_start_;
  QPushButton *p_but_gnss_stop_;
  QTextEdit *p_text_edit_gnss_data_;
  QButtonGroup *p_rbut_grp;
  QRadioButton *p_rbut_gnss;
  QRadioButton *p_rbut_rtk;

  QLabel *p_label_imu_dev_;
  QLabel *p_label_imu_baud_rate_;
  QLineEdit *p_line_edit_imu_dev_;
  QLineEdit *p_line_edit_imu_baud_rate_;
  QPushButton *p_but_imu_start_;
  QPushButton *p_but_imu_stop_;
  QTextEdit *p_text_edit_imu_data_;

  QLabel *p_label_recorder_;
  QLineEdit *p_line_edit_recorder_;
  QPushButton *p_but_recorder_start_;
  QPushButton *p_but_recorder_stop_;
  QTextEdit *p_text_recorder_data_;


  QLabel *p_label_speedo_dev_;
  QLabel *p_label_speedo_baud_rate;
  QLineEdit *p_line_edit_speedo_dev_;
  QLineEdit *p_line_edit_speedo_baud_rate_;
  QPushButton *p_but_speedo_start_;
  QTextEdit *p_text_edit_speedo_data_;

 private:
  std::future<void> future_imu_start_;
  std::future<void> future_imu_text_;
  std::future<void> future_GNSS_start_;
  std::future<void> future_GNSS_text_;
  std::future<void> future_recorcer_start_;
  std::shared_ptr<WheeltecComponent> wheeltecComponent_;
  std::shared_ptr<GnssComponent> gnssComponent_;
  std::shared_ptr<QianxunComponent> qianxunComponent_;
  std::shared_ptr<RecorderComponent> recorderComponent_;

 private:
  int col_pos_[5];
  int row_pos_[5];
  int col_size_[5];
  int row_size_[5];

 Q_SIGNALS:
  void strImuData(QString);
  void strGnssData(QString);
  void strRecordingData(QString);
};

#endif  // MAINWINDOW_H_
