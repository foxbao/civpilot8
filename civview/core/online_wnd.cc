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
#include "online_wnd.h"
#include <QtWidgets/QGridLayout>
#include <memory>
#include <vector>
#include "cyber/cyber.h"
#include "message/drivers/gnss/proto/raw_gnss.pb.h"
#include "message/drivers/imu/proto/imu.pb.h"
#include "vtk_source/source_manager.hpp"
#include "vtk_source/trajectory_source.hpp"

OnlineWnd::OnlineWnd(QWidget *parent) : QDockWidget(parent) {
  using apollo::cyber::service_discovery::TopologyManager;
  channel_manager_ = TopologyManager::Instance()->channel_manager();

  InitUi();
  //监控拓扑变化
  change_conn_ = channel_manager_->AddChangeListener(
      std::bind(&OnlineWnd::TopologyCallback, this, std::placeholders::_1));
  if (!change_conn_.IsConnected()) {
    AERROR << "change connection is not connected";
    return;
  }

  node_ = apollo::cyber::CreateNode("zview");
  if (node_ == nullptr) {
    AERROR << "create node failded, node: quick_recorder";
  }
  ConectSlots();
}

OnlineWnd::~OnlineWnd() {
  channel_manager_->RemoveChangeListener(change_conn_);
}

void OnlineWnd::InitUi() {
  channel_model_ = new QStandardItemModel();
  QStringList headers = {"channel_name", "data_type"};
  channel_model_->setHorizontalHeaderLabels(headers);

  get_all_channels();
  UpdateAllChannel();

  channel_table_view_ = new QTableView(this);
  channel_table_view_->setModel(channel_model_);
  channel_table_view_->setSelectionBehavior(QAbstractItemView::SelectRows);

  channel_table_view_->setSizeAdjustPolicy(
      QAbstractScrollArea::AdjustToContents);
  QWidget *p_main_wiget = new QWidget(this);
  QGridLayout *p_main_grid = new QGridLayout(p_main_wiget);
  p_main_grid->addWidget(channel_table_view_, 0, 0);

  this->setWidget(p_main_wiget);
}

void OnlineWnd::ConectSlots() {
  connect(channel_table_view_, &QAbstractItemView::doubleClicked, this,
          &OnlineWnd::OnRowDoubleClicked);
  connect(this, &OnlineWnd::ChannelChange, this, &OnlineWnd::UpdateAllChannel);
}

void OnlineWnd::TopologyCallback(
    const apollo::cyber::proto::ChangeMsg &change_msg) {
  using namespace apollo::cyber::proto;
  if (change_msg.role_type() == ROLE_WRITER &&
      change_msg.change_type() == CHANGE_CHANNEL) {
    auto const &role_attr = change_msg.role_attr();
    {
      std::lock_guard<std::mutex> lg(mtx_all_channel_);
      switch (change_msg.operate_type()) {
        case OperateType::OPT_JOIN: {
          OnlineChannel channel;
          channel.name_ = role_attr.channel_name();
          channel.type_ = role_attr.message_type();
          all_channel_[role_attr.channel_name()] = channel;
        } break;
        case OperateType::OPT_LEAVE: {
          all_channel_.erase(role_attr.channel_name());
        } break;
      }
    }
    Q_EMIT ChannelChange();
  }
}

void OnlineWnd::get_all_channels() {
  std::vector<apollo::cyber::proto::RoleAttributes> role_attr_vec;
  channel_manager_->GetWriters(&role_attr_vec);
  for (auto &role_attr : role_attr_vec) {
    if (!role_attr.has_channel_name() || role_attr.channel_name().empty()) {
      AWARN << "change message not has a channel name or has an empty one.";
      return;
    }
    if (!role_attr.has_message_type() || role_attr.message_type().empty()) {
      AWARN << "Change message not has a message type or has an empty one.";
      return;
    }
    if (!role_attr.has_proto_desc() || role_attr.proto_desc().empty()) {
      AWARN << "Change message not has a proto desc or has an empty one.";
      return;
    }
    OnlineChannel channel;
    channel.name_ = role_attr.channel_name();
    channel.type_ = role_attr.message_type();
    all_channel_[role_attr.channel_name()] = channel;
  }
}

void OnlineWnd::UpdateAllChannel() {
  channel_model_->removeRows(0, channel_model_->rowCount());
  {
    std::lock_guard<std::mutex> lg(mtx_all_channel_);
    size_t table_row = 0;
    for (auto const &[key, value] : all_channel_) {
      auto *i1 = new QStandardItem{QString::fromStdString(value.name_)};
      auto *i2 = new QStandardItem{QString::fromStdString(value.type_)};
      if (value.is_added_) {
        i1->setBackground(Qt::green);
        i2->setBackground(Qt::green);
      }
      channel_model_->setItem(table_row, 0, i1);
      channel_model_->setItem(table_row, 1, i2);
      ++table_row;
    }
  }
}

void OnlineWnd::OnRowDoubleClicked(const QModelIndex &mi) {
  auto const *data = channel_model_->item(mi.row());
  OnlineChannel one_channel;
  std::string channel_name =
      channel_model_->item(mi.row())->text().toStdString();
  {
    std::lock_guard<std::mutex> lg(mtx_all_channel_);
    if (all_channel_[channel_name].is_added_) {
      all_channel_[channel_name].is_added_ = false;
    } else {
      all_channel_[channel_name].is_added_ = true;
    }
    one_channel = all_channel_[channel_name];
  }
  UpdateAllChannel();
  if (one_channel.name_ == "LocalizationEstimation") {
    auto source = std::make_shared<civ::civview::TrajectorySource>(
        civ::civview::SourceBaseMode::ONLINE, one_channel.name_);
    source->set_max_size(500);
    civ::civview::SourceManager::AddSource(one_channel.name_, source);
    civ::civview::SourceManager::set_main_trj(source);

    using LEProto = civ::localization::LocalizationEstimate;
    using apollo::cyber::ReaderConfig;
    ReaderConfig reader_config;
    reader_config.channel_name = one_channel.name_;
    reader_config.pending_queue_size = 100 * 2;
    all_readers_[one_channel.name_] = node_->CreateReader<LEProto>(
        reader_config, [one_channel, this, source](auto const &msg) {
          source->AppendData(msg);
        });

  } else if (one_channel.name_ == "/WHEELTEC") {
    using IMUProto = civ::drivers::imu::CorrectedImu;
    using apollo::cyber::ReaderConfig;
    ReaderConfig reader_config;
    reader_config.channel_name = one_channel.name_;
    reader_config.pending_queue_size = 100 * 2;

    all_readers_[one_channel.name_] = node_->CreateReader<IMUProto>(
        reader_config, [this](auto const &msg) {});
  }
}
