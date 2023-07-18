/*
 * Copyright (C) 2015 - 2020 QXSI, all rights reserved.
 *
 * This is the demo program for the QXSI PS-SDK.
 * The implementation here is just for reference, please refer to the header
 * file `qxwz_sdk.h` for the detailed definition of the structures and APIs.
 */
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <iostream>
#include <memory>
// // #include <functional>
#include "gnss/R9300/gnss_component.h"
#include "qianxun/qianxun_component.h"
#include "nosr/nosr.h"

void func(std::string gga_upload) {
  // std::cout<<"receive gga upload"<<std::endl;
  g_gga_upload = gga_upload;
  // std::cout << gga_upload << std::endl;
}

using civ::drivers::gnss::GnssComponent;
using civ::drivers::gnss::QianxunComponent;
int main(int argc, char *argv[]) {
  if (argc != 3) {
    std::cout << "Error!\nUsage: " << argv[0] << " DeviceName BaudRate"
              << std::endl;
    return 0;
  }
  std::shared_ptr<QianxunComponent> qianxun = std::make_shared<QianxunComponent>();
  qianxun->Read(argv[1], std::stoul(argv[2]));

  std::shared_ptr<GnssComponent> gnss = std::make_shared<GnssComponent>();
  std::future<void> gnss_thread_;
  std::function<void(std::string)> f = func;
  gnss->SetCallBack(f);
  gnss_thread_ = std::async(std::launch::async, [gnss, argv] {
    gnss->Read(argv[1], std::stoul(argv[2]));
  });

  Nosr nosa;
  nosa.sdk_test(argv[1], std::stoul(argv[2]));
  return 0;
}
