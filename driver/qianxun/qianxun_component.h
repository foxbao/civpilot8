#pragma once
#include <memory>
#include "gnss/R9300/gnss_component.h"
#include "nosr/nosr.h"
namespace civ {
namespace drivers {
namespace gnss {
class QianxunComponent {
 public:
  QianxunComponent();
  ~QianxunComponent() {}
  void Read(const char *device_name,uint32_t baud_rate);
  void Stop();

 private:
  static void func(std::string gga_upload);

 private:
  std::shared_ptr<GnssComponent> gnss;
  std::shared_ptr<Nosr> nosr;
  std::future<void> gnss_thread_;
  std::function<void(std::string)> f;
};
}  // namespace gnss
}  // namespace drivers
}  // namespace civ
