#pragma once

#include <frc2/command/SubsystemBase.h>
#include "str/TwoJointArmDynamics.h"
#include "constants/ArmConstants.h"
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/TalonFXSimCollection.h>
#include <str/ArmConfig.h>
#include <str/KairosInterface.h>
#include <str/ArmPose.h>
#include <frc/controller/ProfiledPIDController.h>

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;

  units::radian_t GetShoulderMotorAngle();
  units::radian_t GetElbowMotorAngle();
  units::radians_per_second_t GetShoulderMotorVelocity();
  units::radians_per_second_t GetElbowMotorVelocity();

  void SetDesiredArmAngles(units::radian_t shoulderAngle, units::radian_t elbowAngle);
  void SetDesiredArmEndAffectorPosition(units::meter_t xPos, units::meter_t yPos, bool shoulderUp = true);

  units::meter_t GetArmEndEffectorSetpointX() const;
  units::meter_t GetArmEndEffectorSetpointY() const;
  bool IsArmAtEndEffectorSetpoint() const;
  bool IsArmAtDesiredAngles() const;
  bool IsArmAtDesiredAngles(const frc::Vectord<2>& desiredState) const;

  frc2::CommandPtr SetDesiredArmEndAffectorPositionFactory(std::function<units::meter_t()> xPos, std::function<units::meter_t()> yPos, std::function<bool()> shoulderUp);
  frc2::CommandPtr SetDesiredArmAnglesFactory(std::function<units::radian_t()> shoulderAngle, std::function<units::radian_t()> elbowAngle);
  frc2::CommandPtr GoToPose(std::function<ArmPose()> closesetPoseToPreset, std::function<ArmPose()> poseToGoTo);
  frc2::CommandPtr GoToPose(std::function<ArmPose()> poseToGoTo);

  frc2::CommandPtr FollowTrajectory(std::function<ArmTrajectoryParams()> trajParams);

  const TwoJointArmDynamics& GetArmSystem() const { return armSystem; };
 private:
  void ConfigureMotors();
  void ResetEncoders();

  ArmPose GetClosestPosePreset();

  int ConvertShoulderAngleToTicks(units::radian_t angle) const;
  int ConvertShoulderVelocityToTicks(units::radians_per_second_t vel) const;
  int ConvertElbowAngleToTicks(units::radian_t angle) const;
  int GetElbowVelocityToTicks(units::radians_per_second_t vel) const;

  void LogStateToAdvantageScope() const;

  ctre::phoenix::motorcontrol::can::WPI_TalonFX shoulderMotor{str::arm_can_ids::SHOULDER_ID};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX elbowMotor{str::arm_can_ids::ELBOW_ID};

  ctre::phoenix::motorcontrol::TalonFXSimCollection shoulderSimCollection{shoulderMotor.GetSimCollection()};
  ctre::phoenix::motorcontrol::TalonFXSimCollection elbowSimCollection{elbowMotor.GetSimCollection()};

  frc::Vectord<6> initialState{0.3338283,-1.9046246268,0,0,0,0};

  units::meter_t currentEndEffectorSetpointX{24_in};
  units::meter_t currentEndEffectorSetpointY{24_in};

  units::radians_per_second_t prevShoulderVel{0_rad_per_s};
  units::radians_per_second_t prevElbowVel{0_rad_per_s};
  
  ArmConfig config{ArmConfig::LoadJson("arm_config.json")};

  bool hasManuallyMoved{false};
  std::string lastRanTrajFinalPoseName{""};

  frc::TrapezoidProfile<units::radians>::Constraints arm_constraints {
    426_deg_per_s,
    99999_deg_per_s_sq
  };

  frc::ProfiledPIDController<units::radians> shoulderPID{0.5, 0, 0, arm_constraints};
  frc::ProfiledPIDController<units::radians> elbowPID{0.5, 0, 0, arm_constraints};
  
  TwoJointArmDynamics armSystem {
    config.shoulder.mass,
    config.elbow.mass,
    config.shoulder.length,
    config.elbow.length,
    config.shoulder.moi,
    config.elbow.moi,
    config.shoulder.cgRadius,
    config.elbow.cgRadius,
    config.shoulder.motor,
    config.elbow.motor,
    config.shoulder.gearReduction,
    config.elbow.gearReduction,
    initialState,
    str::arm_constants::positionTolerance,
    str::arm_constants::velocityTolerance,
    str::arm_constants::controlEffort,
    str::arm_constants::qPos,
    str::arm_constants::qVel,
    str::arm_constants::est,
    str::arm_constants::rPos
  };

  KairosInterface kairos;
  ArmTrajectory trajToFollow{ArmTrajectoryParams{}};
  frc::Timer armTrajTimer;

    // Create a Mechanism2d display of an Arm
  frc::Mechanism2d armDisplay{300, 300};
  frc::MechanismRoot2d* armBase = armDisplay.GetRoot("ArmBase", 150, 150 + 31);
  frc::MechanismLigament2d* armShoulderActual =
    armBase->Append<frc::MechanismLigament2d>(
        "ShoulderActual",
        str::arm_constants::shoulderLength.value(), 
        units::radian_t{armSystem.GetCurrentState()(0)}, 
        6, 
        frc::Color8Bit{frc::Color::kBlue}
    );
  frc::MechanismLigament2d* armElbowActual = 
    armShoulderActual->Append<frc::MechanismLigament2d>(
      "ElbowActual", 
      str::arm_constants::elbowLength.value(),
      units::radian_t{armSystem.GetCurrentState()(1)}, 
      6, 
      frc::Color8Bit{frc::Color::kBlue}
    );

  frc::MechanismRoot2d* tester = armDisplay.GetRoot("TESTER", 150, 150);
  frc::MechanismLigament2d* testerDot =
    tester->Append<frc::MechanismLigament2d>(
        "TESTERDOT",
        1, 
        0_deg, 
        6, 
        frc::Color8Bit{frc::Color::kRed}
    );

  frc::MechanismRoot2d* ekfArm = armDisplay.GetRoot("EKFArm", 150, 150 + 31);
  frc::MechanismLigament2d* armShoulderEkf =
    ekfArm->Append<frc::MechanismLigament2d>(
        "ShoulderEKF",
        str::arm_constants::shoulderLength.value(), 
        units::radian_t{armSystem.GetCurrentState()(0)}, 
        6, 
        frc::Color8Bit{frc::Color::kYellow}
    );
  frc::MechanismLigament2d* armElbowEkf = 
    armShoulderEkf->Append<frc::MechanismLigament2d>(
      "ElbowEKF", 
      str::arm_constants::elbowLength.value(),
      units::radian_t{armSystem.GetCurrentState()(1)}, 
      6, 
      frc::Color8Bit{frc::Color::kYellow}
    );
};
