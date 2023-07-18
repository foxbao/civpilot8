#include "gnss/R9300/gnss_component.h"
#include <vector>
#include "gnss/R9300/parser/R9300_ascii_parser.hpp"

namespace civ {
namespace drivers {
namespace gnss {
using civ::drivers::R9300::R9300AsciiParser;
GnssComponent::GnssComponent(){};
GnssComponent::~GnssComponent() {
  if (serial_) {
    
    delete serial_;
  }
}
bool GnssComponent::Init() {}
int GnssComponent::Read(const char *device_name, uint32_t baud_rate) {
  std::cout << "connecting to " << device_name << ":" << baud_rate << std::endl;
  int remainSize = 0;
  int buff_size = 2048;
  unsigned char buff[buff_size];
  char buff_tmp[buff_size];

  R9300AsciiParser parser;
  serial_ = SerialStream::CreateSerial(device_name, baud_rate);
  if (serial_ && serial_->Connect()) {
    apollo::cyber::Init("gnss_component");
    auto node = apollo::cyber::CreateNode("gnss");
    if (!parser.InitWriters(node)) {
      return -1;
    }

    using STATUS = civ::drivers::common::Stream::Status;
    while (serial_->get_status() == STATUS::CONNECTED) {
      memset(buff_tmp, 0, buff_size);
      memcpy(buff_tmp, buff, remainSize);
      memset(buff + remainSize, 0, buff_size - remainSize);
      // memset(buff,0,buff_size);

      auto length = serial_->read(buff + remainSize, buff_size - remainSize);
      if (length > 0) {
        length += remainSize;

        size_t strlen_length = strlen(reinterpret_cast<char *>(buff));

        if (strlen_length != length) {
          length = strlen_length;
        }

        // output_buff(reinterpret_cast<char *>(buff), length);
        parser.UpdateDataPtr(buff, length);
        std::vector<GNSSmsg> v_gnss;
        parser.ParseData(v_gnss);
        for (const auto &gnss : v_gnss) {
          if (gnss.type_ == GNSSmsgType::GNGGA) {
            if(m_callback)
            {
              m_callback(gnss.to_string().c_str());
            }
            gga_upload_ = gnss.to_string();
          }
        }
        remainSize = parser.RemainSize();
        if (remainSize < length && remainSize > 0) {
          memcpy(buff, buff + length - remainSize, remainSize);
        }
        if (remainSize > 200) {
          std::cout << "remainSize:" << remainSize << std::endl;
        }
      }
    }
  } else {
    return -1;
    std::cout << "cannot connect" << std::endl;
  }
  return 0;
}
void GnssComponent::Stop() {
  if (serial_) {
    serial_->Disconnect();
  }
}
}  // namespace gnss

}  // namespace drivers
}  // namespace civ
