#pragma once

#include <frc/EigenCore.h>
#include <frc/Vector2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <units/dimensionless.h>
#include <units/force.h>
#include <units/math.h>

namespace frc {
  class Force2d {
  public:
    Force2d() = default;
    Force2d(units::newton_t x, units::newton_t y) {
      matrix(0, 0) = x;
      matrix(1, 0) = y;
    };
    Force2d(units::newton_t mag, frc::Rotation2d rotation) : Force2d(mag * rotation.Cos(), mag * rotation.Sin()){};
    Force2d(Eigen::Matrix<units::newton_t, 2, 1> input) {
      matrix = input;
    };
    Force2d(Vector2d<units::newtons> vec) : Force2d(vec.x, vec.y){};
    units::newton_t X() {
      return matrix(0, 0);
    }
    units::newton_t Y() {
      return matrix(1, 0);
    }
    units::newton_t Norm() {
      Eigen::Matrix<double, 2, 1> temp{};
      temp(0, 0) = matrix(0, 0).to<double>();
      temp(1, 0) = matrix(1, 0).to<double>();
      return units::newton_t(temp.norm());
    }
    Vector2d<units::dimensionless::scalar> UnitVector() {
      Vector2d<units::dimensionless::scalar> retVal(this->X() / this->Norm(), this->Y() / this->Norm());
      return retVal;
    };
    Force2d RotateBy(frc::Rotation2d angle) {
      return Force2d(
        this->X() * angle.Cos() - this->Y() * angle.Sin(),
        this->X() * angle.Sin() + this->Y() * angle.Cos()
      );
    }
    Force2d Plus(Force2d other) {
      return Force2d(this->matrix + other.matrix);
    }
    void Accum(Force2d other) {
      this->matrix += other.matrix;
    }
    Force2d Minus(Force2d other) {
      return Force2d(this->matrix - other.matrix);
    }
    Force2d UnaryMinus() {
      Eigen::Matrix<double, 2, 1> temp{};
      temp(0, 0) = matrix(0, 0).to<double>();
      temp(1, 0) = matrix(1, 0).to<double>();
      auto result = temp * -1.0;
      Eigen::Matrix<units::newton_t, 2, 1> retVal{};
      retVal(0, 0) = units::newton_t(result(0, 0));
      retVal(1, 0) = units::newton_t(result(1, 0));
      return Force2d(retVal);
    }
    Force2d Times(units::scalar_t scalar) {
      Eigen::Matrix<double, 2, 1> temp{};
      temp(0, 0) = matrix(0, 0).to<double>();
      temp(1, 0) = matrix(1, 0).to<double>();
      auto result = temp * scalar;
      Eigen::Matrix<units::newton_t, 2, 1> retVal{};
      retVal(0, 0) = units::newton_t(result(0, 0));
      retVal(1, 0) = units::newton_t(result(1, 0));
      return Force2d(retVal);
    }
    Force2d Div(units::scalar_t scalar) {
      Eigen::Matrix<double, 2, 1> temp{};
      temp(0, 0) = matrix(0, 0).to<double>();
      temp(1, 0) = matrix(1, 0).to<double>();
      auto result = temp / scalar;
      Eigen::Matrix<units::newton_t, 2, 1> retVal{};
      retVal(0, 0) = units::newton_t(result(0, 0));
      retVal(1, 0) = units::newton_t(result(1, 0));
      return Force2d(retVal);
    }
    Vector2d<units::newtons> GetVector() {
      Vector2d retVal(this->X(), this->Y());
      return retVal;
    };

  private:
    Eigen::Matrix<units::newton_t, 2, 1> matrix{};
  };

}   