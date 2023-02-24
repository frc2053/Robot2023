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
    retVal.name = "StartingConfig";
    return retVal;
  };

  static constexpr ArmPose StowedConfig() {
    ArmPose retVal;
    retVal.endEffectorPosition = {.5588_m, .2032_m};
    retVal.fromTop = false;
    retVal.name = "StowedConfig";
    return retVal;
  };

  static constexpr ArmPose ScoreConeHigh() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-45_in, 17_in};
    retVal.fromTop = true;
    retVal.name = "ScoreConeHigh";
    return retVal;
  };

  static constexpr ArmPose StraightOut() {
    ArmPose retVal;
    retVal.endEffectorPosition = {51_in, 0_in};
    retVal.fromTop = true;
    retVal.name = "StraightOut";
    return retVal;
  };

  frc::Vectord<2> AsVector() const {
    return frc::Vectord<2>{endEffectorPosition.X().value(), endEffectorPosition.Y().value()};
  };

  frc::Vectord<2> AsJointAngles(const TwoJointArmDynamics& dynamics) const {
    const auto& ikResults = dynamics.CalculateInverseKinematics(frc::Vectord<2>{endEffectorPosition.X(), endEffectorPosition.Y()}, fromTop);
    if(ikResults.has_value()) {
      return ikResults.value();
    }
    else {
      fmt::print("ERROR: A PRESET JOINT POSITION HAS BAD IK!\n");
      return frc::Vectord<2>{0, 0};
    }
  };

  frc::Translation2d endEffectorPosition{0_m, 0_m};
  bool fromTop{true};
  std::string name{""};
};

static inline std::vector<ArmPose> AllPoses{ArmPose::StartingConfig(), ArmPose::ScoreConeHigh(), ArmPose::StowedConfig(), ArmPose::StraightOut()};
