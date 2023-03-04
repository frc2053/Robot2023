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
    retVal.endEffectorPosition = {-47_in, 18_in};
    retVal.fromTop = true;
    retVal.name = "ScoreConeHigh";
    return retVal;
  };

  static constexpr ArmPose ScoreConeMid() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-30.943291_in, 10_in};
    retVal.fromTop = true;
    retVal.name = "ScoreConeMid";
    return retVal;
  }

  static constexpr ArmPose ScorePieceLow() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-19.296905_in, -23.598059_in};
    retVal.fromTop = true;
    retVal.name = "ScorePieceLow";
    return retVal;
  }

  static constexpr ArmPose GroundIntakeFar() {
    ArmPose retVal;
    retVal.endEffectorPosition = {41.75_in, -26.75_in};
    retVal.fromTop = false;
    retVal.name = "GroundIntakeFar";
    return retVal;
  }

  static constexpr ArmPose PlacePieceFromBack() {
    ArmPose retVal;
    retVal.endEffectorPosition = {49_in, 10_in};
    retVal.fromTop = true;
    retVal.name = "PlacePieceFromBack";
    return retVal;
  }

  static constexpr ArmPose IntakeFromSubstation() {
    ArmPose retVal;
    retVal.endEffectorPosition = {-20_in, 8_in};
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
  ArmPose::PlacePieceFromBack(),
  ArmPose::IntakeFromSubstation()
};
