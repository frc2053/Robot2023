#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/EigenCore.h>
#include <str/TwoJointArmDynamics.h>

struct ArmPose {
public:
  static constexpr ArmPose StartingConfig() {
    ArmPose retVal;
    retVal.endEffectorPosition = {20_in, -25_in};
    retVal.isInverted = true;
    return retVal;
  };

  static constexpr ArmPose ScoreConeHigh() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-31_in, 23_in};
    retVal.isInverted = false;
    return retVal;
  };

  frc::Vectord<2> AsVector() const {
    return frc::Vectord<2>{endEffectorPosition.X().value(), endEffectorPosition.Y().value()};
  };

  frc::Vectord<2> AsJointAngles(const TwoJointArmDynamics& dynamics) const {
    return dynamics.CalculateInverseKinematics(frc::Vectord<2>{endEffectorPosition.X(), endEffectorPosition.Y()}, isInverted);
  };

  frc::Translation2d endEffectorPosition{0_m, 0_m};
  bool isInverted{false};
};

static inline std::vector<ArmPose> AllPoses{ArmPose::StartingConfig(), ArmPose::ScoreConeHigh()};
