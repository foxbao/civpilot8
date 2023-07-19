#pragma once
#include <eigen3/Eigen/Core>
#include <vector>
#include "src/inner_types.hpp"
namespace civ {
namespace civmap {

class CivMap {
 public:
  CivMap();
  ~CivMap();
  bool ReadData(std::string file_path);
  std::vector<sp_cZMapLineSegment> get_lines() { return lines_; };
  std::vector<sp_cZMapLineSegment> get_lines_enu();

 private:
  std::vector<sp_cZMapLineSegment> lines_;  // llh
};
DEFINE_EXTEND_TYPE(CivMap);

}  // namespace v2x
}  // namespace coop