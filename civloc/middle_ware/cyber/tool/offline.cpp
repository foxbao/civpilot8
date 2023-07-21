#include <condition_variable>
#include <mutex>
#include <thread>
#include "cyber/cyber.h"
#include "gflags/gflags.h"
#include "middle_ware/cyber/civloc_component.hpp"

#include "cyber_tool/cyber_record_reader.hpp"
DEFINE_string(recorder_file_path,
              "/home/baojiali/Downloads/civpilot/data/202207211457.0000*",
              "recorder file path");

DEFINE_string(dag_file_path, "ddd", "dag file path");

template <typename T>
std::shared_ptr<T> Str2Frame(std::string const content) {
  std::shared_ptr<T> result = std::make_shared<T>();
  result->ParseFromString(content);
  return result;
}
using civ::civloc::CivLocComponent;
using civ::civloc::GNSSProto;
using civ::civloc::IMUProto;
using civ::cyber_tool::CyberRecordReader;
// using civ::civloc::tool::CyberRecordWriter;

void DataVisitor(std::shared_ptr<CivLocComponent> component) {
  CyberRecordReader reader;
  auto civloc_config = component->get_config();
  reader[civloc_config->raw_wheeltec_imu().name()] =
      [component](std::string const content) {
        component->OnGetRawImuFrame(Str2Frame<IMUProto>(content));
      };

  reader["/GNGGA"] = [component](std::string const content) {
    component->OnGetGNGGAFrame(Str2Frame<GNSSProto>(content));
  };
  
  reader.Play(FLAGS_recorder_file_path);
}

int main(int argc, char* argv[]) {
  apollo::cyber::Init("civ_component_offline");
  google::ParseCommandLineFlags(&argc, &argv, true);
  std::cout<<"start offline localizer!"<<std::endl;
  std::cout<<"FLAGS_dag_file_path:"<<FLAGS_dag_file_path<<std::endl;
  std::cout<<"FLAGS_recorder_file_path:"<<FLAGS_recorder_file_path<<std::endl;
  std::shared_ptr<CivLocComponent> civloc =
      std::make_shared<CivLocComponent>(FLAGS_dag_file_path);
  civloc->set_input_data_dir(FLAGS_recorder_file_path);

  DataVisitor(civloc);
}
