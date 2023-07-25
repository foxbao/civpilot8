#include "cyber/cyber.h"
#include "middle_ware/cyber/civloc_component.hpp"

DEFINE_string(dag_file_path, "ddd", "dag file path");

using apollo::cyber::Reader;
using civ::civloc::CivLocComponent;


int main(int argc, char *argv[])
{
  // init cyber framework
  apollo::cyber::Init(argv[0]);


  gflags::ParseCommandLineFlags(&argc, &argv, true);
  std::cout << "FLAGS_dag_file_path:" << FLAGS_dag_file_path << std::endl;
  std::shared_ptr<CivLocComponent> civloc =
      std::make_shared<CivLocComponent>(FLAGS_dag_file_path);

  // need the system to hold down for the data reading session to read data
  apollo::cyber::WaitForShutdown();
  return 0;
}