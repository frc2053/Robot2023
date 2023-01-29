#pragma once

#include <frc2/command/SubsystemBase.h>
#include "str/TwoJointArmDynamics.h"
#include "constants/ArmConstants.h"
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <ctre/phoenix/motorcontrol/TalonFXSimCollection.h>

class ArmSubsystem : public frc2::SubsystemBase {
 public:
  ArmSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;
  void SetDesiredArmAngles(units::radian_t shoulderAngle, units::radian_t elbowAngle);
  void SetDesiredArmEndAffectorPosition(units::meter_t xPos, units::meter_t yPos);
  units::meter_t GetArmEndEffectorSetpointX();
  units::meter_t GetArmEndEffectorSetpointY();
  frc2::CommandPtr SetDesiredArmEndAffectorPositionFactory(std::function<units::meter_t()> xPos, std::function<units::meter_t()> yPos);
 private:
  void ConfigureMotors();
  void ResetEncoders();
  units::radian_t GetShoulderMotorAngle();
  units::radian_t GetElbowMotorAngle();
  units::radians_per_second_t GetShoulderMotorVelocity();
  units::radians_per_second_t GetElbowMotorVelocity();

  int ConvertShoulderAngleToTicks(units::radian_t angle);
  int ConvertShoulderVelocityToTicks(units::radians_per_second_t vel);
  int ConvertElbowAngleToTicks(units::radian_t angle);
  int GetElbowVelocityToTicks(units::radians_per_second_t vel);

  ctre::phoenix::motorcontrol::can::WPI_TalonFX shoulderMotorLeft{str::arm_can_ids::LEFT_SHOULDER_ID};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX shoulderMotorRight{str::arm_can_ids::RIGHT_SHOULDER_ID};
  ctre::phoenix::motorcontrol::can::WPI_TalonFX elbowMotor{str::arm_can_ids::ELBOW_ID};

  ctre::phoenix::motorcontrol::TalonFXSimCollection shoulderSimCollection{shoulderMotorLeft.GetSimCollection()};
  ctre::phoenix::motorcontrol::TalonFXSimCollection elbowSimCollection{elbowMotor.GetSimCollection()};

  frc::Vectord<6> initialState{0,0,0,0,0,0};

  units::meter_t currentEndEffectorSetpointX{42_in};
  units::meter_t currentEndEffectorSetpointY{24_in};

  TwoJointArmDynamics armSystem {
    str::arm_constants::shoulderMass,
    str::arm_constants::elbowMass,
    str::arm_constants::shoulderLength,
    str::arm_constants::elbowLength,
    str::arm_constants::shoulderMoi,
    str::arm_constants::elbowMoi,
    str::arm_constants::cogDistShoulder,
    str::arm_constants::cogDistElbow,
    str::arm_constants::shoulderGearbox,
    str::arm_constants::elbowGearbox,
    str::arm_constants::shoulderGearing,
    str::arm_constants::elbowGearing,
    initialState,
    str::arm_constants::positionTolerance,
    str::arm_constants::velocityTolerance,
    str::arm_constants::controlEffort,
    str::arm_constants::qPos,
    str::arm_constants::qVel,
    str::arm_constants::est,
    str::arm_constants::rPos
  };

    // Create a Mechanism2d display of an Arm
  frc::Mechanism2d armDisplay{300, 300};
  frc::MechanismRoot2d* armBase = armDisplay.GetRoot("ArmBase", 150, 150);
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

  frc::MechanismRoot2d* ekfArm = armDisplay.GetRoot("EKFArm", 150, 150);
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
