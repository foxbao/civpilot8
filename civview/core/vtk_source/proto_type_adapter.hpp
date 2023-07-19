#pragma once

#include <google/protobuf/message.h>
#include "common/util/type_define.hpp"

namespace civ {
namespace civview {
using namespace civ::common::util;
using Msg = google::protobuf::Message;
DEFINE_EXTEND_TYPE(Msg);

// 统一时间戳的访问
struct Msg2 {
 public:
  size_t get_t0_ms() const { return static_cast<size_t>(t0_ * 1e3); }
  size_t get_t1_ms() const { return static_cast<size_t>(t1_ * 1e3); }

 public:
  double t0_;  // 秒
  double t1_;  // 秒
  sp_cMsg msg_;
};

DEFINE_EXTEND_TYPE(Msg2);

}  // namespace civview
}  // namespace civ
