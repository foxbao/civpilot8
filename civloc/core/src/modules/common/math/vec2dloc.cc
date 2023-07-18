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

#include "vec2dloc.h"
namespace civ {
namespace civloc {
Vec2dloc Vec2dloc::CreateUnitVec2dloc(const double angle) {
  return Vec2dloc(std::cos(angle), std::sin(angle));
}

double Vec2dloc::Length() const { return std::hypot(x_, y_); }

double Vec2dloc::LengthSquare() const { return x_ * x_ + y_ * y_; }

double Vec2dloc::Angle() const { return std::atan2(y_, x_); }

void Vec2dloc::Normalize() {
  const double l = Length();
  if (l > kMathEpsilon) {
    x_ /= l;
    y_ /= l;
  }
}

double Vec2dloc::DistanceTo(const Vec2dloc &other) const {
  return std::hypot(x_ - other.x_, y_ - other.y_);
}

double Vec2dloc::DistanceSquareTo(const Vec2dloc &other) const {
  const double dx = x_ - other.x_;
  const double dy = y_ - other.y_;
  return dx * dx + dy * dy;
}

double Vec2dloc::CrossProd(const Vec2dloc &other) const {
  return x_ * other.y() - y_ * other.x();
}

double Vec2dloc::InnerProd(const Vec2dloc &other) const {
  return x_ * other.x() + y_ * other.y();
}

Vec2dloc Vec2dloc::rotate(const double angle) const {
  return Vec2dloc(x_ * cos(angle) - y_ * sin(angle),
               x_ * sin(angle) + y_ * cos(angle));
}

void Vec2dloc::SelfRotate(const double angle) {
  double tmp_x = x_;
  x_ = x_ * cos(angle) - y_ * sin(angle);
  y_ = tmp_x * sin(angle) + y_ * cos(angle);
}

Vec2dloc Vec2dloc::operator+(const Vec2dloc &other) const {
  return Vec2dloc(x_ + other.x(), y_ + other.y());
}

Vec2dloc Vec2dloc::operator-(const Vec2dloc &other) const {
  return Vec2dloc(x_ - other.x(), y_ - other.y());
}

Vec2dloc Vec2dloc::operator*(const double ratio) const {
  return Vec2dloc(x_ * ratio, y_ * ratio);
}

Vec2dloc Vec2dloc::operator/(const double ratio) const {
  // CHECK_GT(std::abs(ratio), kMathEpsilon);
  return Vec2dloc(x_ / ratio, y_ / ratio);
}

Vec2dloc &Vec2dloc::operator+=(const Vec2dloc &other) {
  x_ += other.x();
  y_ += other.y();
  return *this;
}

Vec2dloc &Vec2dloc::operator-=(const Vec2dloc &other) {
  x_ -= other.x();
  y_ -= other.y();
  return *this;
}

Vec2dloc &Vec2dloc::operator*=(const double ratio) {
  x_ *= ratio;
  y_ *= ratio;
  return *this;
}

Vec2dloc &Vec2dloc::operator/=(const double ratio) {
  // CHECK_GT(std::abs(ratio), kMathEpsilon);
  x_ /= ratio;
  y_ /= ratio;
  return *this;
}

bool Vec2dloc::operator==(const Vec2dloc &other) const {
  return (std::abs(x_ - other.x()) < kMathEpsilon &&
          std::abs(y_ - other.y()) < kMathEpsilon);
}

Vec2dloc operator*(const double ratio, const Vec2dloc &vec) { return vec * ratio; }


}  // namespace math
}  // namespace common
