/******************************************************************************
 * Copyright 2018 The Zhito-AI Authors. All Rights Reserved.
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

#include "common/coordinate_transform/proj4_transform.h"

#include <string>

namespace civ {
namespace common {
namespace coord_transform {

bool FrameTransform::LatlonToUtmXY(double lon_rad, double lat_rad,
                                   UTMCoor *utm_xy) {
  projPJ pj_latlon;
  projPJ pj_utm;
  int zone = 0;
  zone = static_cast<int>((lon_rad * RAD_TO_DEG + 180) / 6) + 1;
  std::string latlon_src =
      "+proj=longlat +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +no_defs";
  std::ostringstream utm_dst;
  utm_dst << "+proj=utm +zone=" << zone << " +ellps=WGS84 +units=m +no_defs";
  if (!(pj_latlon = pj_init_plus(latlon_src.c_str()))) {
    return false;
  }
  if (!(pj_utm = pj_init_plus(utm_dst.str().c_str()))) {
    return false;
  }
  double longitude = lon_rad;
  double latitude = lat_rad;
  pj_transform(pj_latlon, pj_utm, 1, 1, &longitude, &latitude, nullptr);
  utm_xy->x = longitude;
  utm_xy->y = latitude;
  pj_free(pj_latlon);
  pj_free(pj_utm);
  return true;
}
bool FrameTransform::UtmXYToLatlon(double x, double y, int zone, bool southhemi,
                                   WGS84Corr *latlon) {
  projPJ pj_latlon;
  projPJ pj_utm;
  std::string latlon_src =
      "+proj=longlat +ellps=WGS84 +towgs84=0,0,0,0,0,0,0 +no_defs";
  std::ostringstream utm_dst;
  utm_dst << "+proj=utm +zone=" << zone << " +ellps=WGS84 +units=m +no_defs";
  if (!(pj_latlon = pj_init_plus(latlon_src.c_str()))) {
    return false;
  }
  if (!(pj_utm = pj_init_plus(utm_dst.str().c_str()))) {
    return false;
  }
  pj_transform(pj_utm, pj_latlon, 1, 1, &x, &y, nullptr);
  latlon->log = x;
  latlon->lat = y;
  pj_free(pj_latlon);
  pj_free(pj_utm);
  return true;
}

bool FrameTransform::XYZToBlh(const Vector3d &xyz, Vector3d *blh) {
  projPJ pj_xyz;
  projPJ pj_blh;
  std::string xyz_src = "+proj=geocent +datum=WGS84";
  std::string blh_dst = "+proj=latlong +datum=WGS84";
  if (!(pj_xyz = pj_init_plus(xyz_src.c_str()))) {
    return false;
  }
  if (!(pj_blh = pj_init_plus(blh_dst.c_str()))) {
    return false;
  }
  double x = xyz[0];
  double y = xyz[1];
  double z = xyz[2];
  pj_transform(pj_xyz, pj_blh, 1, 1, &x, &y, &z);
  (*blh)[0] = x;
  (*blh)[1] = y;
  (*blh)[2] = z;
  pj_free(pj_xyz);
  pj_free(pj_blh);
  return true;
}
bool FrameTransform::BlhToXYZ(const Vector3d &blh, Vector3d *xyz) {
  projPJ pj_xyz;
  projPJ pj_blh;
  std::string blh_src = "+proj=latlong +datum=WGS84";
  std::string xyz_dst = "+proj=geocent +datum=WGS84";

  if (!(pj_blh = pj_init_plus(blh_src.c_str()))) {
    return false;
  }
  if (!(pj_xyz = pj_init_plus(xyz_dst.c_str()))) {
    return false;
  }
  double longitude = blh[0];
  double latitude = blh[1];
  double height = blh[2];
  pj_transform(pj_blh, pj_xyz, 1, 1, &longitude, &latitude, &height);
  (*xyz)[0] = longitude;
  (*xyz)[1] = latitude;
  (*xyz)[2] = height;
  pj_free(pj_xyz);
  pj_free(pj_blh);
  return true;
}

// TEST(FrameTransformTestSuite, LatlonToUtmXYTest) {
//   double lon_rad = -2.129343746458001;
//   double lat_rad = 0.6530018835651807;
//   UTMCoor utm_xy;
//   EXPECT_TRUE(FrameTransform::LatlonToUtmXY(lon_rad, lat_rad, &utm_xy));
//   EXPECT_LT(std::fabs(utm_xy.x - 588278.9834174265), 1e-5);
//   EXPECT_LT(std::fabs(utm_xy.y - 4141295.255870659), 1e-5);
// }

// TEST(FrameTransformTestSuite, UtmXYToLatlonTest) {
//   double x = 588278.9834174265;
//   double y = 4141295.255870659;
//   int zone = 10;
//   bool southhemi = false;
//   WGS84Corr latlon;
//   EXPECT_TRUE(FrameTransform::UtmXYToLatlon(x, y, zone, southhemi, &latlon));
//   EXPECT_LT(std::fabs(latlon.log + 2.129343746458001), 1e-5);
//   EXPECT_LT(std::fabs(latlon.lat - 0.6530018835651807), 1e-5);
// }

}  // namespace coord_transform
}  // namespace common
}  // namespace civ
