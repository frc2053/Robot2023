#pragma once

#include "constants/SwerveConstants.h"
#include "str/SparkMaxWrapper.h"
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <rev/SparkMaxRelativeEncoder.h>
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
    void SimulationPeriodic();
    units::ampere_t GetDriveMotorCurrent();
    units::ampere_t GetSteerMotorCurrent();
    void Characterize(units::volt_t driveVoltage);

  private:
    void ConfigureRotationMotor();
    void ConfigureDriveMotor();

    frc::SimpleMotorFeedforward<units::meters> driveFF{
      str::swerve_drive_consts::DRIVE_KS,
      str::swerve_drive_consts::DRIVE_KV,
      str::swerve_drive_consts::DRIVE_KA
    };

    str::SparkMaxWrapper steerMotor;
    str::SparkMaxWrapper driveMotor;

    bool isVoltageCompensating{false};
    units::radian_t moduleAngleOffset;
  };
}
