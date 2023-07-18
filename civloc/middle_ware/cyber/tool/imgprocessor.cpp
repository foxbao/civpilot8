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

#include "middle_ware/cyber/tool/imgprocessor.hpp"
#include <vector>
#include "common/coordinate_transform/earth.hpp"


namespace civ {
namespace civloc {
IMGPROCESSOR::IMGPROCESSOR() {
  scale_ = 3;
  H_ = 1000;
  W_ = 1000;
  result_img_ = cv::Mat(H_, W_, CV_32FC3);
  img_folder_ = "img_result";
  PlotAxis(result_img_);
}

IMGPROCESSOR::~IMGPROCESSOR() {}

void IMGPROCESSOR::PlotSaveStateLLH(sp_cState state, std::string type,
                                    cv::Scalar color, bool is_deg) {
  std::string file_path =
      img_folder_ + "/" + std::to_string(state->t0_) + type + ".jpg";
  PlotPosLLH(result_img_, state->pos_, color, is_deg);
  cv::imwrite(file_path, result_img_);
}

void IMGPROCESSOR::PlotPosLLH(cv::Mat &img, const Vec3d &pos_llh,
                              cv::Scalar color, bool is_deg) {
  using namespace civ::common::coord_transform;
  Vec3d pos_enu = Earth::LLH2ENU(pos_llh, is_deg);
  Vec3d pos_IMG = Convert2IMG(pos_enu);
  cv::circle(result_img_, cv::Point(pos_IMG[0], pos_IMG[1]), 5, color, -1);
}

void IMGPROCESSOR::PlotPositionCovariance() {
  double sigma_a = 8.42;
  double sigma_b = 0.9387;
  double s = 5.991;

  double startAngle = 0;
  double endAngle = 360;
  double angle = 0;

  Vec3d pos_enu{100, 200};
  Vec3d pos_IMG = Convert2IMG(pos_enu);

  cv::ellipse(result_img_, cv::Point(pos_IMG(0), pos_IMG(1)),
              cv::Size(200, 400),  // ssss
              angle, startAngle, endAngle, cv::Scalar(255, 255, 255), 5);

  cv::imwrite("ellipse.jpg", result_img_);
  // Eigen::Mat a = Mat::ones(2,2,CV_32FC1);
  //     a.at<float>(0,0) = 5;
  //     a.at<float>(0,1) = -4;
  //     a.at<float>(1,0) = -4;
  //     a.at<float>(1,1) = 5;
  // Mat eigen_values;
  // Mat eigen_vector;

  // eigen(a,eigen_values,eigen_vector);
}

void IMGPROCESSOR::PlotRawGNSS(sp_cZGnss gnss, bool is_deg) {
  std::string file_path =
      img_folder_ + "/" + std::to_string(gnss->t0_) + "_gnss.jpg";
  PlotPosLLH(result_img_, gnss->pos_, cv::Scalar(255, 0, 0));
  // cv::imwrite(file_path, result_img_);
}

Vec2dloc IMGPROCESSOR::Convert2IMG(const Vec2dloc &pt) {
  Vec2dloc pt_IMG;
  pt_IMG.set_x(pt.x() * scale_);
  pt_IMG.set_y(-pt.y() * scale_);
  pt_IMG.set_x(pt_IMG.x() + (W_ / 2));
  pt_IMG.set_y(pt_IMG.y() + (H_ / 2));
  return pt_IMG;
}

Vec3d IMGPROCESSOR::Convert2IMG(const Vec3d &pos_enu) {
  Vec3d pos_enu_IMG;
  pos_enu_IMG[0] = pos_enu[0];
  pos_enu_IMG[1] = -pos_enu[1];
  pos_enu_IMG[2] = pos_enu[2];

  pos_enu_IMG[0] += W_ / 2;
  pos_enu_IMG[1] += H_ / 2;
  return pos_enu_IMG;
}

void IMGPROCESSOR::PlotAxis(cv::Mat &img) {
  cv::Point center(W_ / 2, H_ / 2);
  cv::Point eastEnd(W_, H_ / 2);
  cv::Point northEnd(W_ / 2, 0);
  cv::line(img, center, eastEnd, cv::Scalar(0, 255, 0), 1);
  cv::line(img, center, northEnd, cv::Scalar(0, 255, 0), 1);

  auto font = cv::FONT_HERSHEY_SIMPLEX;
  int thickness = 2;
  Vec2dloc label_100_0(100, 0);
  Vec2dloc label_100_0_IMG = Convert2IMG(label_100_0);
  cv::Point point_label_100_0_IMG(label_100_0_IMG.x(), label_100_0_IMG.y());
  cv::putText(img, "100m", point_label_100_0_IMG, font, 0.5,
              cv::Scalar(255, 255, 255), thickness);

  Vec2dloc label_0_100(0, 100);
  Vec2dloc label_0_100_IMG = Convert2IMG(label_0_100);
  cv::Point point_label_0_100_IMG(label_0_100_IMG.x(), label_0_100_IMG.y());
  cv::putText(img, "100m", point_label_0_100_IMG, font, 0.5,
              cv::Scalar(255, 255, 255), thickness);

  Vec2dloc GNSS_label(-100, 120);
  Vec2dloc GNSS_label_IMG = Convert2IMG(GNSS_label);
  cv::Point pos_GNSS_label_IMG(GNSS_label_IMG.x(), GNSS_label_IMG.y());
  cv::putText(img, "GNSS", pos_GNSS_label_IMG, font, 2.0, cv::Scalar(255, 0, 0),
              thickness);

  Vec2dloc IMU_label(-100, 100);
  Vec2dloc IMU_label_IMG = Convert2IMG(IMU_label);
  cv::Point pos_IMU_label_IMG(IMU_label_IMG.x(), IMU_label_IMG.y());
  cv::putText(img, "IMU", pos_IMU_label_IMG, font, 2.0, cv::Scalar(0, 255, 255),
              thickness);

  Vec2dloc Fuse_label(-100, 80);
  Vec2dloc Fuse_label_IMG = Convert2IMG(Fuse_label);
  cv::Point pos_Fuse_label_IMG(Fuse_label_IMG.x(), Fuse_label_IMG.y());
  cv::putText(img, "Fuse", pos_Fuse_label_IMG, font, 2.0, cv::Scalar(0, 0, 255),
              thickness);
}
}  // namespace civloc
}  // namespace civ
