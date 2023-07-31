/******************************************************************************
 * Copyright 2022 The CIV Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once
#include <iostream>
#include <string>
#include <opencv2/opencv.hpp>
#include "modules/common/inner_types.hpp"
#include "modules/common/math/vec2dloc.h"

using civ::civloc::Vec2dloc;
namespace civ {
namespace civloc {

class IMGPROCESSOR {
 public:
  IMGPROCESSOR();
  ~IMGPROCESSOR();
  void PlotSaveStateLLH(sp_cState state, std::string type = "",
                        cv::Scalar color = cv::Scalar(0, 255, 255),
                        bool is_deg = false);
  void PlotRawGNSS(sp_cZGnss gnss, bool is_deg = false);
  void PlotPositionCovariance();

 private:
  void PlotPosLLH(cv::Mat &img, const Vec3d &pos_llh,
                  cv::Scalar color = cv::Scalar(0, 255, 255),
                  bool is_deg = false);
  Vec3d Convert2IMG(const Vec3d &pos_enu);
  Vec2dloc Convert2IMG(const Vec2dloc &pt);

 private:
  cv::Mat result_img_;
  int scale_;
  int H_;
  int W_;
  std::string img_folder_;
  double pos_ori_deg_[3];
  double pos_ori_rad_[3];
  void PlotAxis(cv::Mat &img);
};
DEFINE_EXTEND_TYPE(IMGPROCESSOR);
}  // namespace civloc
}  // namespace civ
