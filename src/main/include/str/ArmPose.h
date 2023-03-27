#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/EigenCore.h>
#include <str/TwoJointArmDynamics.h>
#include <frc/DataLogManager.h>

struct ArmPose {
public:

  static constexpr ArmPose StartingConfig() {
    ArmPose retVal;
    retVal.endEffectorPosition = {7.0_in, 11.5_in};
    retVal.fromTop = false;
    retVal.name = "StartingConfig";
    return retVal;
  };

  static constexpr ArmPose ScoreConeHigh() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-43.4304_in, 20.2_in};
    retVal.fromTop = true;
    retVal.name = "ScoreConeHigh";
    return retVal;
  };

  static constexpr ArmPose ScoreConeMid() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-30.5_in, 10.5_in};
    retVal.fromTop = true;
    retVal.name = "ScoreConeMid";
    return retVal;
  }

  static constexpr ArmPose ScorePieceLow() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-16.610274_in, -28.431469_in};
    retVal.fromTop = true;
    retVal.name = "ScorePieceLow";
    return retVal;
  }

  static constexpr ArmPose GroundIntakeFar() {
    ArmPose retVal;
    retVal.endEffectorPosition = {38.5799_in, -21.6341_in};
    retVal.fromTop = false;
    retVal.name = "GroundIntakeFar";
    return retVal;
  }

  static constexpr ArmPose IntakeFromSubstation() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-17.340059_in, 15.500553_in};
    retVal.fromTop = true;
    retVal.name = "IntakeFromSubstation";
    return retVal;
  }

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
      frc::DataLogManager::Log(fmt::format("ERROR: A PRESET JOINT POSITION HAS BAD IK!"));
      return frc::Vectord<2>{0, 0};
    }
  };

  frc::Translation2d endEffectorPosition{0_m, 0_m};
  bool fromTop{true};
  std::string name{""};
};

static inline std::vector<ArmPose> AllPoses{
  ArmPose::StartingConfig(),
  ArmPose::ScoreConeHigh(),
  ArmPose::StraightOut(),
  ArmPose::ScoreConeMid(),
  ArmPose::ScorePieceLow(),
  ArmPose::GroundIntakeFar(),
  ArmPose::IntakeFromSubstation()
};
