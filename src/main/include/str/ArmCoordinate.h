// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include "units/length.h"

namespace str {
class ArmCoordinate {
 public:
  /**
   * Constructs a ArmCoordinate with X and Y components equal to zero.
   */
  constexpr ArmCoordinate() = default;

  /**
   * Constructs a ArmCoordinate with the X and Y components equal to the
   * provided values.
   *
   * @param x The x component of the translation.
   * @param y The y component of the translation.
   */
  constexpr ArmCoordinate(units::meter_t x, units::meter_t y);

  /**
   * Constructs a ArmCoordinate with the provided distance and angle. This is
   * essentially converting from polar coordinates to Cartesian coordinates.
   *
   * @param distance The distance from the origin to the end of the translation.
   * @param angle The angle between the x-axis and the translation vector.
   */
  constexpr ArmCoordinate(units::meter_t distance, const frc::Rotation2d& angle);

  /**
   * Calculates the distance between two translations in 2D space.
   *
   * The distance between translations is defined as √((x₂−x₁)²+(y₂−y₁)²).
   *
   * @param other The translation to compute the distance to.
   *
   * @return The distance between the two translations.
   */
  units::meter_t Distance(const ArmCoordinate& other) const;

  /**
   * Returns the X component of the translation.
   *
   * @return The X component of the translation.
   */
  constexpr units::meter_t X() const { return m_x; }

  /**
   * Returns the Y component of the translation.
   *
   * @return The Y component of the translation.
   */
  constexpr units::meter_t Y() const { return m_y; }

  /**
   * Returns the norm, or distance from the origin to the translation.
   *
   * @return The norm of the translation.
   */
  units::meter_t Norm() const;

  /**
   * Returns the angle this translation forms with the positive X axis.
   *
   * @return The angle of the translation
   */
  constexpr frc::Rotation2d Angle() const;

  /**
   * Applies a rotation to the translation in 2D space.
   *
   * This multiplies the translation vector by a counterclockwise rotation
   * matrix of the given angle.
   *
   * <pre>
   * [x_new]   [other.cos, -other.sin][x]
   * [y_new] = [other.sin,  other.cos][y]
   * </pre>
   *
   * For example, rotating a ArmCoordinate of &lt;2, 0&gt; by 90 degrees will
   * return a ArmCoordinate of &lt;0, 2&gt;.
   *
   * @param other The rotation to rotate the translation by.
   *
   * @return The new rotated translation.
   */
  constexpr ArmCoordinate RotateBy(const frc::Rotation2d& other) const;

  /**
   * Returns the sum of two translations in 2D space.
   *
   * For example, Translation3d{1.0, 2.5} + Translation3d{2.0, 5.5} =
   * Translation3d{3.0, 8.0}.
   *
   * @param other The translation to add.
   *
   * @return The sum of the translations.
   */
  constexpr ArmCoordinate operator+(const ArmCoordinate& other) const;

  /**
   * Returns the difference between two translations.
   *
   * For example, ArmCoordinate{5.0, 4.0} - ArmCoordinate{1.0, 2.0} =
   * ArmCoordinate{4.0, 2.0}.
   *
   * @param other The translation to subtract.
   *
   * @return The difference between the two translations.
   */
  constexpr ArmCoordinate operator-(const ArmCoordinate& other) const;

  /**
   * Returns the inverse of the current translation. This is equivalent to
   * rotating by 180 degrees, flipping the point over both axes, or negating all
   * components of the translation.
   *
   * @return The inverse of the current translation.
   */
  constexpr ArmCoordinate operator-() const;

  /**
   * Returns the translation multiplied by a scalar.
   *
   * For example, ArmCoordinate{2.0, 2.5} * 2 = ArmCoordinate{4.0, 5.0}.
   *
   * @param scalar The scalar to multiply by.
   *
   * @return The scaled translation.
   */
  constexpr ArmCoordinate operator*(double scalar) const;

  /**
   * Returns the translation divided by a scalar.
   *
   * For example, ArmCoordinate{2.0, 2.5} / 2 = ArmCoordinate{1.0, 1.25}.
   *
   * @param scalar The scalar to divide by.
   *
   * @return The scaled translation.
   */
  constexpr ArmCoordinate operator/(double scalar) const;

  /**
   * Checks equality between this ArmCoordinate and another object.
   *
   * @param other The other object.
   * @return Whether the two objects are equal.
   */
  bool operator==(const ArmCoordinate& other) const;

  constexpr bool operator<(const ArmCoordinate& other) const;

 private:
  units::meter_t m_x = 0_m;
  units::meter_t m_y = 0_m;
};

}  // namespace str

#include "ArmCoordinate.inc"