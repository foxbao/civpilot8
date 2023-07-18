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
#include <eigen3/Eigen/Core>
#include <vector>
#include <string>
namespace civ {
namespace civloc {
constexpr double kMathEpsilon = 1e-10;
class Vec2dloc {
 public:
  //! Constructor which takes x- and y-coordinates.
  constexpr Vec2dloc(const double x, const double y) noexcept : x_(x), y_(y) {}

  //! Constructor returning the zero vector.
  constexpr Vec2dloc() noexcept : Vec2dloc(0, 0) {}

  //! Creates a unit-vector with a given angle to the positive x semi-axis
  static Vec2dloc CreateUnitVec2dloc(const double angle);

  //! Getter for x component
  double x() const { return x_; }

  //! Getter for y component
  double y() const { return y_; }

  //! Setter for x component
  void set_x(const double x) { x_ = x; }

  //! Setter for y component
  void set_y(const double y) { y_ = y; }

  //! Gets the length of the vector
  double Length() const;

  //! Gets the squared length of the vector
  double LengthSquare() const;

  //! Gets the angle between the vector and the positive x semi-axis
  double Angle() const;

  //! Returns the unit vector that is co-linear with this vector
  void Normalize();

  //! Returns the distance to the given vector
  double DistanceTo(const Vec2dloc &other) const;

  //! Returns the squared distance to the given vector
  double DistanceSquareTo(const Vec2dloc &other) const;

  //! Returns the "cross" product between these two Vec2dloc (non-standard).
  double CrossProd(const Vec2dloc &other) const;

  //! Returns the inner product between these two Vec2dloc.
  double InnerProd(const Vec2dloc &other) const;

  //! rotate the vector by angle.
  Vec2dloc rotate(const double angle) const;

  //! rotate the vector itself by angle.
  void SelfRotate(const double angle);

  //! Sums two Vec2dloc
  Vec2dloc operator+(const Vec2dloc &other) const;

  //! Subtracts two Vec2dloc
  Vec2dloc operator-(const Vec2dloc &other) const;

  //! Multiplies Vec2dloc by a scalar
  Vec2dloc operator*(const double ratio) const;

  //! Divides Vec2dloc by a scalar
  Vec2dloc operator/(const double ratio) const;

  //! Sums another Vec2dloc to the current one
  Vec2dloc &operator+=(const Vec2dloc &other);

  //! Subtracts another Vec2dloc to the current one
  Vec2dloc &operator-=(const Vec2dloc &other);

  //! Multiplies this Vec2dloc by a scalar
  Vec2dloc &operator*=(const double ratio);

  //! Divides this Vec2dloc by a scalar
  Vec2dloc &operator/=(const double ratio);

  //! Compares two Vec2dloc
  bool operator==(const Vec2dloc &other) const;

  //! Returns a human-readable string representing this object
  std::string DebugString() const;

 protected:
  double x_ = 0.0;
  double y_ = 0.0;
};

//! Multiplies the given Vec2dloc by a given scalar
Vec2dloc operator*(const double ratio, const Vec2dloc &vec);
}  // namespace math
}  // namespace common
