#pragma once
#include <string>

namespace civ {
namespace drivers {

void output_buff(char *buff, int size) {
  for (int i = 0; i < size; i++) {
    printf("%c", buff[i]);
  }
  printf("\n");
}

std::string binaryToHex(const unsigned char *data, int length) {
  std::string ret;
  static const char *hex = "0123456789ABCDEF";
  for (int i = 0; i < length; i++) {
    ret.push_back(hex[(data[i] >> 4) & 0xf]);  // 取二进制高四位
    ret.push_back(hex[data[i] & 0xf]);         // 取二进制低四位
  }
  return ret;
}

std::string binaryToHex(uint8_t data) {
  std::string ret;
  static const char *hex = "0123456789ABCDEF";
  ret.push_back(hex[(data >> 4) & 0xf]);  // 取二进制高四位
  ret.push_back(hex[data & 0xf]);         // 取二进制低四位
  return ret;
}
}  // namespace drivers
}  // namespace civ
