#include "middle_ware/cyber/civloc_gflags.hpp"

namespace civ {
namespace civloc {
DEFINE_string(sensors_setting_path,
              "/zhito/zloc/middle_ware/cyber/conf/sensors_setting.pb.txt",
              "sensors_setting_path");
DEFINE_string(locator_setting_path,
              "/zhito/zloc/middle_ware/cyber/conf/locator.pb.txt",
              "locator_setting_path");
}  // namespace civloc
}  // namespace civ
