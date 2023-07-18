#pragma once
#include <functional>
#include <memory>
#include <string>
#include "common/stream/serial_stream.h"
#include "cyber/cyber.h"
#include "message/drivers/gnss/proto/raw_gnss.pb.h"

namespace civ {
namespace drivers {
namespace gnss {
using apollo::cyber::Component;
using civ::drivers::common::SerialStream;
class GnssComponent : public Component<> {
 public:
  GnssComponent();
  ~GnssComponent();
  bool Init() override;
  int Read(const char *device_name, uint32_t baud_rate);
  void Stop();
  void SetCallBack(const std::function<void(const char* res)> &cb) {
    m_callback = cb;
  }

 private:
  SerialStream *serial_ = nullptr;
  std::string gga_upload_ = "";  // the gga to upload to qianxun basestation
  std::function<void(const char* res)> m_callback;
  std::shared_ptr<apollo::cyber::Writer<civ::drivers::rawgnss::GNGGA>>
      gnss_writer_ = nullptr;
};
}  // namespace gnss
}  // namespace drivers
}  // namespace civ
