#pragma once

#include "constants/SwerveConstants.h"
#include "str/SparkMaxSwerveWrapper.h"
#include <ctre/phoenix/motorcontrol/can/WPI_TalonFX.h>
#include <frc/Timer.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>

namespace str {
  class SwerveModule {
  public:
    SwerveModule(int driveCanId, int rotationCanId, units::radian_t moduleAngle);
    frc::SwerveModuleState GetState();
    frc::SwerveModulePosition GetPosition();
    void SetDesiredState(const frc::SwerveModuleState& referenceState, bool openLoop, bool voltageComp);
    void ResetEncoders();
    units::volt_t GetDriveAppliedVoltage();
    units::volt_t GetRotationAppliedVoltage();
    void SetSimState(units::radian_t steerPos, units::meter_t drivePos, units::meters_per_second_t driveVel, units::ampere_t driveCurrent, units::ampere_t steerCurrent);
    void SimulationPeriodic();
    units::ampere_t GetDriveMotorCurrent();
    units::ampere_t GetSteerMotorCurrent();

  private:
    frc::SimpleMotorFeedforward<units::meters> driveFF{
      str::swerve_drive_consts::DRIVE_KS,
      str::swerve_drive_consts::DRIVE_KV,
      str::swerve_drive_consts::DRIVE_KA
    };

    str::SparkMaxSwerveWrapper steerMotor;
    str::SparkMaxSwerveWrapper driveMotor;

    units::meters_per_second_t prevModuleSpeed{0};
    units::second_t prevTime{0};
    frc::Timer ffTimer;

    bool isVoltageCompensating{false};
    units::radian_t moduleAngleOffset;
  };
}
