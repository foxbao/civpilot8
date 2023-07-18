#include "qianxun/qianxun_component.h"

namespace civ {
namespace drivers {
namespace gnss {
QianxunComponent::QianxunComponent() {
  gnss = std::make_shared<GnssComponent>();
  nosr = std::make_shared<Nosr>();
  f = this->func;
  gnss->SetCallBack(f);
}
void QianxunComponent::Read(const char *device_name, uint32_t baud_rate) {
  gnss_thread_ = std::async(std::launch::async, [this, device_name, baud_rate] {
    this->gnss->Read(device_name, baud_rate);
  });

  nosr->sdk_test(device_name, baud_rate);
}

void QianxunComponent::Stop() {
  this->gnss->Stop();
}

void QianxunComponent::func(std::string gga_upload) {
  // std::cout<<"receive gga upload"<<std::endl;
  g_gga_upload = gga_upload;
  // std::cout << gga_upload << std::endl;
}

}  // namespace gnss
}  // namespace drivers
}  // namespace civ