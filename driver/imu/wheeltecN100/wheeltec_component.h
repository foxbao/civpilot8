#pragma once
// #include <iostream>
#include <memory>
#include <string>
#include "common/stream/serial_stream.h"
#include "cyber/cyber.h"
#include "message/drivers/imu/proto/imu.pb.h"

namespace civ {
namespace drivers {
namespace wheeltec {
using apollo::cyber::Component;
using civ::drivers::common::SerialStream;
class WheeltecComponent : public Component<> {
 public:
  WheeltecComponent();
  ~WheeltecComponent();
  bool Init() override;
  int Read(const char *device_name, uint32_t baud_rate);
  void Stop();
  void SetCallBack(const std::function<void(const char* res)> &cb);

 private:
  SerialStream *serial_ = nullptr;
  std::shared_ptr<apollo::cyber::Writer<civ::drivers::imu::CorrectedImu>>
      imu_writer_ = nullptr;
  int serial_data_size_;
  std::string current_data_hex_;
  std::function<void(const char* res)> m_callback;
};
}  // namespace wheeltec
}  // namespace drivers
}  // namespace civ
