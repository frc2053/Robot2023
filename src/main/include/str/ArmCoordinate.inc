// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "ArmCoordinate.h"
#include "units/length.h"

namespace str {

constexpr ArmCoordinate::ArmCoordinate(units::meter_t x, units::meter_t y)
    : m_x(x), m_y(y) {}

constexpr ArmCoordinate::ArmCoordinate(units::meter_t distance,
                                       const frc::Rotation2d& angle)
    : m_x(distance * angle.Cos()), m_y(distance * angle.Sin()) {}

constexpr frc::Rotation2d ArmCoordinate::Angle() const {
  return frc::Rotation2d{m_x.value(), m_y.value()};
}

constexpr ArmCoordinate ArmCoordinate::RotateBy(const frc::Rotation2d& other) const {
  return {m_x * other.Cos() - m_y * other.Sin(),
          m_x * other.Sin() + m_y * other.Cos()};
}

constexpr ArmCoordinate ArmCoordinate::operator+(
    const ArmCoordinate& other) const {
  return {X() + other.X(), Y() + other.Y()};
}

constexpr ArmCoordinate ArmCoordinate::operator-(
    const ArmCoordinate& other) const {
  return *this + -other;
}

constexpr ArmCoordinate ArmCoordinate::operator-() const {
  return {-m_x, -m_y};
}

constexpr ArmCoordinate ArmCoordinate::operator*(double scalar) const {
  return {scalar * m_x, scalar * m_y};
}

constexpr ArmCoordinate ArmCoordinate::operator/(double scalar) const {
  return operator*(1.0 / scalar);
}

constexpr bool ArmCoordinate::operator<(const ArmCoordinate& other) const {
  return (this->m_x < other.m_x) && (this->m_y < other.m_y);
}

}  // namespace str