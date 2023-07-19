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
#include <QtGui/QStandardItemModel>
#include <QtWidgets/QDockWidget>
#include <QtWidgets/QTableView>
#include <QtWidgets/QTableWidgetItem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include "cyber/base/signal.h"
#include "cyber/common/file.h"
#include "cyber/common/log.h"
#include "cyber/cyber.h"
#include "cyber/node/node.h"
#include "cyber/proto/role_attributes.pb.h"
#include "cyber/proto/topology_change.pb.h"
#include "cyber/record/record_reader.h"
#include "cyber/service_discovery/specific_manager/channel_manager.h"
#include "cyber/service_discovery/topology_manager.h"

#include "vtk_source/source_base.hpp"
struct OnlineChannel {
  std::string name_;
  std::string type_;
  bool is_added_ = false;
};

class OnlineWnd : public QDockWidget {
  Q_OBJECT
 public:
  explicit OnlineWnd(QWidget *parent = nullptr);
  ~OnlineWnd();

 private:
  void InitUi();
  void ConectSlots();
 private Q_SLOTS:
  void OnRowDoubleClicked(const QModelIndex &mi);

 private:
  // table
  QStandardItemModel *channel_model_ = nullptr;
  QTableView *channel_table_view_ = nullptr;
  std::map<std::string, OnlineChannel> all_channel_;
  std::mutex mtx_all_channel_;

  apollo::cyber::base::Connection<const apollo::cyber::proto::ChangeMsg &>
      change_conn_;
  std::shared_ptr<apollo::cyber::service_discovery::ChannelManager>
      channel_manager_ = nullptr;
  void TopologyCallback(const apollo::cyber::proto::ChangeMsg
                            &change_msg);  // 拓扑变化时的回调函数
  void get_all_channels();
  void UpdateAllChannel();

 private:
 // node_用于创建reader
  std::shared_ptr<apollo::cyber::Node> node_ = nullptr;
  std::map<std::string, std::shared_ptr<apollo::cyber::ReaderBase>>
      all_readers_;
 public:
  std::map<std::string, OnlineChannel> &get_channels() { return all_channel_; }


 Q_SIGNALS:
  void ChannelChange();
};
