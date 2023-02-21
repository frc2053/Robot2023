#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/EigenCore.h>
#include <str/TwoJointArmDynamics.h>

struct ArmPose {
public:
  static constexpr ArmPose StartingConfig() {
    ArmPose retVal;
    retVal.endEffectorPosition = {20_in, -25_in};
    retVal.fromTop = true;
    return retVal;
  };

  static constexpr ArmPose StowedConfig() {
    ArmPose retVal;
    retVal.endEffectorPosition = {.5588_m, .2032_m};
    retVal.fromTop = false;
    return retVal;
  };

  static constexpr ArmPose ScoreConeHigh() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-45_in, 17_in};
    retVal.fromTop = true;
    return retVal;
  };

  frc::Vectord<2> AsVector() const {
    return frc::Vectord<2>{endEffectorPosition.X().value(), endEffectorPosition.Y().value()};
  };

  frc::Vectord<2> AsJointAngles(const TwoJointArmDynamics& dynamics) const {
    return dynamics.CalculateInverseKinematics(frc::Vectord<2>{endEffectorPosition.X(), endEffectorPosition.Y()}, fromTop);
  };

  frc::Translation2d endEffectorPosition{0_m, 0_m};
  bool fromTop{true};
};

static inline std::vector<ArmPose> AllPoses{ArmPose::StartingConfig(), ArmPose::ScoreConeHigh(), ArmPose::StowedConfig()};
