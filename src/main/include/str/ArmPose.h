#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/EigenCore.h>

struct ArmPose {
public:
  static constexpr ArmPose StartingConfig() {
    ArmPose retVal;
    retVal.endEffectorPosition = {20_in, -25_in};
    return retVal;
  };

  static constexpr ArmPose ScoreConeHigh() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-31_in, 23_in};
    return retVal;
  };

  frc::Vectord<2> AsVector() {
    return frc::Vectord<2>{endEffectorPosition.X().value(), endEffectorPosition.Y().value()};
  };

  frc::Vectord<2> AsJointAngles(const TwoJointArmDynamics& dynamics, bool invert) {
    return dynamics.CalculateInverseKinematics(frc::Vectord<2>{endEffectorPosition.X(), endEffectorPosition.Y()}, invert);
  }

  frc::Translation2d endEffectorPosition{0_m, 0_m};
};