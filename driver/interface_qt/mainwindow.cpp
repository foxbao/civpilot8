/* Copyright
lallala
dafdasfd */

#include "mainwindow.h"
#include <QDebug>
#include <QLineEdit>
#include <QProcess>
#include <QPushButton>
#include <QString>
#include <QThread>
#include <QVBoxLayout>
#include <QWidget>
#include <iostream>
#include <memory>
#include <string>
#include "./ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent)
    : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  qDebug() << "主线程对象地址:  " << QThread::currentThread();

  InitReader();
  InitRecorder();
  InitUi();
}

MainWindow::~MainWindow() { delete ui; }

void MainWindow::executeLinuxCmd(QString strCmd, QString& strResult,
                                 QString& strErrorInfo) {
  QProcess p;
  p.start("bash", QStringList() << "-c" << strCmd);
  p.waitForFinished();
  strResult = p.readAllStandardOutput();
  strErrorInfo = p.readAllStandardError();
}

void MainWindow::InitReader() {
  wheeltecComponent_ = std::make_shared<WheeltecComponent>();
  gnssComponent_ = std::make_shared<GnssComponent>();
  qianxunComponent_ = std::make_shared<QianxunComponent>();
}
void MainWindow::InitRecorder() {
  recorderComponent_ = std::make_shared<RecorderComponent>();
}

void MainWindow::InitUi() {
  this->resize(1000, 500);
  this->setWindowTitle("Data Receiving System");

  this->InitUIPos();
  this->InitUiGNSS();
  this->InitUiIMU();
  this->InitUiRecorder();
}

void MainWindow::InitUIPos() {
  // position of every column
  col_pos_[0] = 20;
  col_pos_[1] = col_pos_[0] + 200;
  col_pos_[2] = col_pos_[0] + 400;
  col_pos_[3] = col_pos_[0] + 570;
  col_pos_[4] = col_pos_[0] + 750;

  row_pos_[0] = 50;
  row_pos_[1] = row_pos_[0] + 100;
  row_pos_[2] = row_pos_[0] + 200;
  row_pos_[3] = row_pos_[0] + 300;
  row_pos_[4] = row_pos_[0] + 400;

  col_size_[0] = 180;
  col_size_[1] = 180;
  col_size_[2] = 150;
  col_size_[3] = 150;
  col_size_[4] = 200;

  row_size_[0] = 50;
  row_size_[1] = 50;
  row_size_[2] = 50;
  row_size_[3] = 50;
  row_size_[4] = 50;
}

void MainWindow::InitUiGNSS() {
  p_label_gnss_dev_ = new QLabel(this);
  p_label_gnss_dev_->setGeometry(col_pos_[0], row_pos_[0] - 20, col_size_[0],
                                 row_size_[0] - 30);
  p_label_gnss_dev_->setText("Gnss Dev");

  p_line_edit_gnss_dev_ = new QLineEdit(this);
  p_line_edit_gnss_dev_->setGeometry(col_pos_[0], row_pos_[0], col_size_[0],
                                     row_size_[0]);
  p_line_edit_gnss_dev_->setText("/dev/ttyUSB1");

  p_label_gnss_baud_rate_ = new QLabel(this);
  p_label_gnss_baud_rate_->setGeometry(col_pos_[1], row_pos_[0] - 20,
                                       col_size_[0], row_size_[0] - 30);
  p_label_gnss_baud_rate_->setText("Gnss Baud Rate");

  p_line_edit_gnss_baud_rate_ = new QLineEdit(this);
  p_line_edit_gnss_baud_rate_->setGeometry(col_pos_[1], row_pos_[0],
                                           col_size_[1], row_size_[0]);
  p_line_edit_gnss_baud_rate_->setText("115200");

  p_but_gnss_start_ = new QPushButton(this);
  p_but_gnss_start_->setGeometry(col_pos_[2], row_pos_[0], col_size_[2],
                                 row_size_[0]);  //
  p_but_gnss_start_->setText("Start Gnss");

  p_but_gnss_stop_ = new QPushButton(this);
  p_but_gnss_stop_->setGeometry(col_pos_[3], row_pos_[0], col_size_[3],
                                row_size_[0]);  //
  p_but_gnss_stop_->setText("Stop Gnss");

  p_text_edit_gnss_data_ = new QTextEdit(this);
  p_text_edit_gnss_data_->setGeometry(col_pos_[4], row_pos_[0], col_size_[4],
                                      row_size_[0] + 30);
  p_text_edit_gnss_data_->setText("GNSS Data Received:");

  p_rbut_grp = new QButtonGroup(this);
  p_rbut_gnss = new QRadioButton("GNSS Mode", this);
  p_rbut_gnss->setGeometry(col_pos_[2], row_pos_[0] - 30, col_size_[2],
                           row_size_[0] - 20);
  p_rbut_rtk = new QRadioButton("RTK Mode", this);
  p_rbut_rtk->setGeometry(col_pos_[3], row_pos_[0] - 30, col_size_[3],
                          row_size_[0] - 20);

  p_rbut_grp->addButton(p_rbut_gnss, 0);
  p_rbut_grp->addButton(p_rbut_rtk, 1);
  p_rbut_gnss->setChecked(1);

  QObject::connect(p_but_gnss_start_, &QPushButton::clicked, this,
                   &MainWindow::StartGnss);

  QObject::connect(p_but_gnss_stop_, &QPushButton::clicked, this,
                   &MainWindow::StopGnss);
  QObject::connect(this, SIGNAL(strGnssData(QString)), this,
                   SLOT(ShowGnssData(QString)));
}

void MainWindow::InitUiIMU() {
  p_label_imu_dev_ = new QLabel(this);
  p_label_imu_dev_->setGeometry(col_pos_[0], row_pos_[1] - 20, col_size_[0],
                                row_size_[1] - 30);
  p_label_imu_dev_->setText("IMU Dev");

  p_line_edit_imu_dev_ = new QLineEdit(this);
  p_line_edit_imu_dev_->setGeometry(col_pos_[0], row_pos_[1], col_size_[0],
                                    row_size_[1]);
  p_line_edit_imu_dev_->setText("/dev/ttyUSB0");

  p_label_imu_baud_rate_ = new QLabel(this);
  p_label_imu_baud_rate_->setGeometry(col_pos_[1], row_pos_[1] - 20,
                                      col_size_[0], row_size_[1] - 30);
  p_label_imu_baud_rate_->setText("IMU Baud Rate");

  p_line_edit_imu_baud_rate_ = new QLineEdit(this);
  p_line_edit_imu_baud_rate_->setGeometry(col_pos_[1], row_pos_[1],
                                          col_size_[1], row_size_[1]);
  p_line_edit_imu_baud_rate_->setText("1000000");

  p_but_imu_start_ = new QPushButton(this);
  p_but_imu_start_->setGeometry(col_pos_[2], row_pos_[1], col_size_[2],
                                row_size_[1]);  //
  p_but_imu_start_->setText("Start IMU");

  p_but_imu_stop_ = new QPushButton(this);
  p_but_imu_stop_->setGeometry(col_pos_[3], row_pos_[1], col_size_[3],
                               row_size_[1]);  //
  p_but_imu_stop_->setText("Stop IMU");

  p_text_edit_imu_data_ = new QTextEdit(this);
  p_text_edit_imu_data_->setGeometry(col_pos_[4], row_pos_[1], col_size_[4],
                                     row_size_[1] + 30);
  p_text_edit_imu_data_->setText("IMU Data Received:");

  QObject::connect(p_but_imu_start_, &QPushButton::clicked, this,
                   &MainWindow::StartIMU);
  QObject::connect(p_but_imu_stop_, &QPushButton::clicked, this,
                   &MainWindow::StopIMU);
  QObject::connect(this, SIGNAL(strImuData(QString)), this,
                   SLOT(ShowIMUData(QString)));
}

void MainWindow::InitUiRecorder() {
  p_label_recorder_ = new QLabel(this);
  p_label_recorder_->setGeometry(col_pos_[0], row_pos_[3] - 20, col_size_[0],
                                 row_size_[1] - 30);
  p_label_recorder_->setText("Recorder File");

  p_line_edit_recorder_ = new QLineEdit(this);
  p_line_edit_recorder_->setGeometry(col_pos_[0], row_pos_[3], col_size_[0] * 2,
                                     row_size_[3]);
  p_line_edit_recorder_->setText("202207281515");

  p_but_recorder_start_ = new QPushButton(this);
  p_but_recorder_start_->setGeometry(col_pos_[2], row_pos_[3], col_size_[2],
                                     row_size_[3]);
  p_but_recorder_start_->setText("Start Record");

  p_but_recorder_stop_ = new QPushButton(this);
  p_but_recorder_stop_->setGeometry(col_pos_[3], row_pos_[3], col_size_[3],
                                    row_size_[3]);
  p_but_recorder_stop_->setText("Stop Record");

  p_text_recorder_data_ = new QTextEdit(this);

  p_text_recorder_data_->setGeometry(col_pos_[4], row_pos_[3], col_size_[4],
                                     row_size_[3] + 30);
  p_text_recorder_data_->setText("recording:");

  QObject::connect(p_but_recorder_start_, &QPushButton::clicked, this,
                   &MainWindow::StartRecorder);
  QObject::connect(p_but_recorder_stop_, &QPushButton::clicked, this,
                   &MainWindow::StopRecorder);
  QObject::connect(this, SIGNAL(strRecordingData(QString)), this,
                   SLOT(ShowRecordingData(QString)));
}

void MainWindow::StartGnss() {
  std::string baud_rate = p_line_edit_gnss_baud_rate_->text().toStdString();
  std::string device = p_line_edit_gnss_dev_->text().toStdString();
  p_text_edit_gnss_data_->setText("connecting:" +
                                  p_line_edit_gnss_dev_->text());
  // start gnss data reading
  if (p_rbut_gnss->isChecked()) {
    if (this->gnssComponent_) {
      this->gnssComponent_->SetCallBack([this](const char* res) {
        emit strGnssData(QString::fromStdString(res));
      });
      future_GNSS_start_ =
          std::async(std::launch::async, [this, device, baud_rate] {
            int ret = this->gnssComponent_->Read(device.c_str(),
                                                 std::stoul(baud_rate));
          });
    }
  }
  // start rtk reading
  if (p_rbut_rtk->isChecked()) {
    if (this->qianxunComponent_) {
      future_GNSS_start_ = std::async(std::launch::async, [this, device,
                                                           baud_rate] {
        this->qianxunComponent_->Read(device.c_str(), std::stoul(baud_rate));
      });
    }
  }
}

void MainWindow::StopGnss() {
  p_text_edit_gnss_data_->setText("disconnected");
  if (this->gnssComponent_) {
    this->gnssComponent_->Stop();
  }
}

void MainWindow::ShowGnssData(QString gnss_data) {
  p_text_edit_gnss_data_->setText(gnss_data);
}

void MainWindow::StartIMU() {
  p_text_edit_imu_data_->setText(p_line_edit_imu_dev_->text());
  std::string baud_rate = p_line_edit_imu_baud_rate_->text().toStdString();
  std::string device = p_line_edit_imu_dev_->text().toStdString();
  if (this->wheeltecComponent_) {
    this->wheeltecComponent_->SetCallBack([this](const char* res) {
      emit strImuData(QString::fromStdString(res));
    });
    future_imu_start_ =
        std::async(std::launch::async, [this, device, baud_rate] {
          this->wheeltecComponent_->Read(device.c_str(), std::stoul(baud_rate));
        });

  } else {
    p_text_edit_imu_data_->setText("not connected");
  }
}

void MainWindow::StopIMU() {
  p_text_edit_imu_data_->setText("disconnected");
  if (this->wheeltecComponent_) {
    this->wheeltecComponent_->Stop();
  }
}

void MainWindow::ShowIMUData(QString imu_data) {
  p_text_edit_imu_data_->setText(imu_data);
}

void MainWindow::StartRecorder() {
  // to do
  if (this->recorderComponent_)
  {
  }
  std::string file_name = p_line_edit_recorder_->text().toStdString();
}

void MainWindow::StopRecorder() {
  // to do
}

void MainWindow::ShowRecordingData(QString record_data) { int ddd = 2; }
