#pragma once

#include <type_traits>
#include <units/dimensionless.h>

namespace frc {
  template<typename T>
  struct Vector2d {
    static_assert(units::traits::is_unit<T>::value, "T must be a unit!");
    Vector2d() = default;
    Vector2d(units::unit_t<T> x, units::unit_t<T> y) {
      this->x = x;
      this->y = y;
    };

    void Rotate(units::radian_t angle) {
      double cosA = units::math::cos(angle);
      double sinA = units::math::sin(angle);
      units::unit_t<T> out[2];
      out[0] = (x * cosA) - (y * sinA);
      out[1] = (x * sinA) + (y * cosA);
      x = out[0];
      y = out[1];
    };

    units::unit_t<T> Dot(const Vector2d<units::scalar>& vec) const {
      return (x * vec.x) + (y * vec.y);
    }

    units::unit_t<T> Magnitude() const {
      return std::sqrt((x * x) + (y * y));
    };

    units::scalar_t ScalarProject(const Vector2d<T>& vec) const {
      return Dot(vec) / vec.Magnitude();
    };

    template<typename V>
    units::unit_t<units::compound_unit<T, V>> Cross(const Vector2d<V>& vec) const {
      return (x * vec.y) - (y * vec.x);
    }

    units::unit_t<T> x{};
    units::unit_t<T> y{};
  };

}   